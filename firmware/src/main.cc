#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <bsp/board_api.h>
#include <tusb.h>

#ifdef ADC_ENABLED
#include <hardware/adc.h>
#endif
#include <hardware/flash.h>
#include <hardware/gpio.h>
#include <pico/bootrom.h>
#include <pico/mutex.h>
#include <pico/platform.h>
#include <pico/stdio.h>
#include <pico/unique_id.h>

#include "activity_led.h"
#include "config.h"
#include "crc.h"
#include "descriptor_parser.h"
#include "globals.h"
#include "i2c.h"
#include "mcp4651.h"
#include "our_descriptor.h"
#include "platform.h"
#include "remapper.h"
#include "tick.h"

// [이 코드 블록을 main.cc 상단에 삽입하세요]

// ---▼ KVM 호환성 v7 수정: '스마트 조립기' 버퍼 ▼---
// '엔진'이 분리해서 보낸 데이터를 조립하기 위한 전역 버퍼

// 'v2 통합 광고지'의 구조: 키보드(16) + 마우스(9) = 25 바이트
#define KVM_REPORT_SIZE 25 

// 1. 최종 '통합 버퍼' (KVM/윈도우로 전송될)
static uint8_t kvm_combined_report[KVM_REPORT_SIZE] = { 0 };

// 2. '엔진'이 보낸 '키보드(ID 2)' 데이터를 임시 저장할 버퍼
static uint8_t last_kb_report[16] = { 0 }; // 16 바이트 (NKRO)

// 3. '엔진'이 보낸 '마우스(ID 1)' 데이터를 임시 저장할 버퍼
static uint8_t last_mouse_report[9] = { 0 }; // 9 바이트 (수평 휠 포함)
// ---▲ KVM 호환성 v7 수정 끝 ▲---

// RP2350 UF2s wipe the last sector of flash every time
// because of RP2350-E10 errata mitigation. So we put
// the config one sector down.
#if PICO_RP2350
#define CONFIG_OFFSET_IN_FLASH (PICO_FLASH_SIZE_BYTES - PERSISTED_CONFIG_SIZE - 4096)
#else
#define CONFIG_OFFSET_IN_FLASH (PICO_FLASH_SIZE_BYTES - PERSISTED_CONFIG_SIZE)
#endif

#define FLASH_CONFIG_IN_MEMORY (((uint8_t*) XIP_BASE) + CONFIG_OFFSET_IN_FLASH)

#define ADC_USAGE_PAGE 0xFFF80000

uint64_t next_print = 0;

mutex_t mutexes[(uint8_t) MutexId::N];

uint32_t gpio_valid_pins_mask = 0;
uint32_t gpio_in_mask = 0;
uint32_t gpio_out_mask = 0;
uint32_t prev_gpio_state = 0;
uint64_t last_gpio_change[32] = { 0 };
bool set_gpio_dir_pending = false;

#ifdef ADC_ENABLED
uint16_t prev_adc_state[NADCS] = { 0 };
#endif

void print_stats_maybe() {
    uint64_t now = time_us_64();
    if (now > next_print) {
        print_stats();
        while (next_print < now) {
            next_print += 1000000;
        }
    }
}

void __no_inline_not_in_flash_func(sof_handler)(uint32_t frame_count) {
    sof_callback();
}

// [do_send_report 함수 전체를 이 코드로 교체]

bool do_send_report(uint8_t interface, const uint8_t* report_with_id, uint8_t len) {
    uint8_t report_id = report_with_id[0];
    const uint8_t* report_data = report_with_id + 1;
    uint8_t data_len = len - 1;

    // ... (interface != 0 로직은 그대로) ...
    if (interface != 0) { 
        tud_hid_n_report(interface, report_id, report_data, data_len);
        return true; 
    }

    if (report_id == 2) { // ID 2: '키보드' 데이터가 도착
        // [수정] 데이터가 0이 아닐 때만 버퍼에 복사
        bool non_zero = false;
        for(int i=0; i<data_len; i++) {
            if (report_data[i] != 0) {
                non_zero = true;
                break;
            }
        }
        if (non_zero && data_len <= 16) {
            memcpy(last_kb_report, report_data, data_len);
        }
        
    } else if (report_id == 1) { // ID 1: '마우스' 데이터가 도착
        // [수정] 데이터가 0이 아닐 때만 버퍼에 복사
        bool non_zero = false;
        for(int i=0; i<data_len; i++) {
            if (report_data[i] != 0) {
                non_zero = true;
                break;
            }
        }
        if (non_zero && data_len <= 9) {
            memcpy(last_mouse_report, report_data, data_len);
        }
    }
    return true; 
}

void gpio_pins_init() {
    gpio_valid_pins_mask = get_gpio_valid_pins_mask();
    gpio_init_mask(gpio_valid_pins_mask);
}

void set_gpio_inout_masks(uint32_t in_mask, uint32_t out_mask) {
    // if some pin appears as both input and output, input wins
    gpio_out_mask = (out_mask & ~in_mask) & gpio_valid_pins_mask;
    // we treat all pins except the output ones as input so that the monitor works
    gpio_in_mask = gpio_valid_pins_mask & ~gpio_out_mask;
    set_gpio_dir_pending = true;
}

void set_gpio_dir() {
    gpio_set_dir_masked(gpio_in_mask, 0);
    // output pin direction will be set in write_gpio()
    for (uint8_t i = 0; i <= 29; i++) {
        uint32_t bit = 1 << i;
        if (gpio_valid_pins_mask & bit) {
            gpio_set_pulls(i, gpio_in_mask & bit, false);
        }
    }
}

#ifdef ADC_ENABLED
void adc_pins_init() {
    adc_init();
    for (int n = 26; n < 26 + NADCS; n++) {
        adc_gpio_init(n);
    }

#ifdef PICO_SMPS_MODE_PIN
    // (This only does anything on a Pico, but won't hurt on custom board v8.)
    gpio_init(PICO_SMPS_MODE_PIN);
    gpio_set_dir(PICO_SMPS_MODE_PIN, GPIO_OUT);
    gpio_put(PICO_SMPS_MODE_PIN, true);
#endif
}
#endif

bool read_gpio(uint64_t now) {
    uint32_t gpio_state = gpio_get_all() & gpio_in_mask;
    uint32_t changed = prev_gpio_state ^ gpio_state;
    if (changed != 0) {
        for (uint8_t i = 0; i <= 29; i++) {
            uint32_t bit = 1 << i;
            if (changed & bit) {
                if (last_gpio_change[i] + gpio_debounce_time <= now) {
                    uint32_t usage = GPIO_USAGE_PAGE | i;
                    int32_t state = !(gpio_state & bit);  // active low
                    set_input_state(usage, state, state);
                    if (monitor_enabled) {
                        monitor_usage(usage, state, 0);
                    }
                    last_gpio_change[i] = now;
                } else {
                    // ignore this change
                    gpio_state ^= bit;
                    changed ^= bit;
                }
            }
        }
        prev_gpio_state = gpio_state;
    }
    return changed != 0;
}

void write_gpio() {
    if (suspended) {
        return;
    }

    uint32_t value = gpio_out_state[0] | (gpio_out_state[1] << 8) | (gpio_out_state[2] << 16) | (gpio_out_state[3] << 24);
    switch (gpio_output_mode) {
        case 0:
            gpio_put_masked(gpio_out_mask, value);
            gpio_set_dir_masked(gpio_out_mask, gpio_out_mask);
            break;
        case 1:
            gpio_put_masked(gpio_out_mask, 0);
            gpio_set_dir_masked(gpio_out_mask, value);
            break;
    }
    memset(gpio_out_state, 0, sizeof(gpio_out_state));
}

#ifdef ADC_ENABLED
bool read_adc() {
    bool changed = false;
    for (int i = 0; i < NADCS; i++) {
        adc_select_input(i);
        uint16_t state = adc_read();
        if (state != prev_adc_state[i]) {
            changed = true;
            prev_adc_state[i] = state;
        }
        uint32_t usage = ADC_USAGE_PAGE | i;
        set_input_state(usage, state, state >> 4);
        if (monitor_enabled) {
            monitor_usage(usage, state, 0);
        }
    }
    return changed;
}
#endif

void do_persist_config(uint8_t* buffer) {
#if !PICO_COPY_TO_RAM
    uint32_t ints = save_and_disable_interrupts();
#endif
    flash_range_erase(CONFIG_OFFSET_IN_FLASH, PERSISTED_CONFIG_SIZE);
    flash_range_program(CONFIG_OFFSET_IN_FLASH, buffer, PERSISTED_CONFIG_SIZE);
#if !PICO_COPY_TO_RAM
    restore_interrupts(ints);
#endif
}

void reset_to_bootloader() {
    reset_usb_boot(0, 0);
}

void pair_new_device() {
}

void clear_bonds() {
}

void my_mutexes_init() {
    for (int i = 0; i < (int8_t) MutexId::N; i++) {
        mutex_init(&mutexes[i]);
    }
}

void my_mutex_enter(MutexId id) {
    mutex_enter_blocking(&mutexes[(uint8_t) id]);
}

void my_mutex_exit(MutexId id) {
    mutex_exit(&mutexes[(uint8_t) id]);
}

uint64_t get_time() {
    return time_us_64();
}

uint64_t get_unique_id() {
    pico_unique_board_id_t unique_id;
    pico_get_unique_board_id(&unique_id);
    uint64_t ret = 0;
    for (int i = 0; i < 8; i++) {
        ret |= (uint64_t) unique_id.id[7 - i] << (8 * i);
    }
    return ret;
}

int main() {
    my_mutexes_init();
    gpio_pins_init();
#ifdef I2C_ENABLED
    our_i2c_init();
#endif
#ifdef ADC_ENABLED
    adc_pins_init();
#endif
    tick_init();
    load_config(FLASH_CONFIG_IN_MEMORY);
    our_descriptor = &our_descriptors[our_descriptor_number];
    parse_our_descriptor();
    set_mapping_from_config();
    board_init();
    extra_init();
    tusb_init();
    stdio_init_all();

    tud_sof_isr_set(sof_handler);

    next_print = time_us_64() + 1000000;

    while (true) {
        bool tick;
        bool new_report;
        read_report(&new_report, &tick);
        if (new_report) {
            activity_led_on();
        }
        if (their_descriptor_updated) {
            update_their_descriptor_derivates();
            their_descriptor_updated = false;
        }
        if (tick) {
            bool gpio_state_changed = read_gpio(time_us_64());
            if (gpio_state_changed) {
                activity_led_on();
            }
#ifdef ADC_ENABLED
            read_adc();
#endif
            process_mapping(true);
            write_gpio();
#ifdef MCP4651_ENABLED
            mcp4651_write();
#endif
        }
        tud_task();
        if (boot_protocol_updated) {
            parse_our_descriptor();
            boot_protocol_updated = false;
            config_updated = true;
        }
        if (resume_pending) {
            resume_pending = false;
            suspended = false;
        }
        if (config_updated) {
            set_mapping_from_config();
            config_updated = false;
        }
        if (set_gpio_dir_pending && !suspended) {
            set_gpio_dir();
            set_gpio_dir_pending = false;
        }
        //if (tud_hid_n_ready(0)) {
        //    send_report(do_send_report);
        //}
       // if (monitor_enabled && tud_hid_n_ready(1)) {
       //     send_monitor_report(do_send_report);
       // }
        // ---▼ KVM 호환성 v8 수정: '메인 루프 조립기' ▼---

// 1. '엔진' 큐에 쌓인 '분리된' 리포트들을 모두 처리합니다.
//    (이 함수는 'v8' do_send_report를 호출하여 'last_kb_report'와 'last_mouse_report' 버퍼를 '최신'으로 채웁니다)
send_report(do_send_report);

// 2. KVM/윈도우(인터페이스 0)가 '전송 준비'가 되었는지 확인합니다.
if (tud_hid_n_ready(0)) {
    // 'v7'에서 실패한 '조립'을 여기서, '메인 루프'가 직접 수행합니다.

    // 2a. '통합 버퍼'의 0번 칸부터 '최신' 키보드 데이터를 채웁니다.
    memcpy(kvm_combined_report, last_kb_report, 16);

    // 2b. '통합 버퍼'의 16번 칸부터 '최신' 마우스 데이터를 채웁니다.
    memcpy(kvm_combined_report + 16, last_mouse_report, 9);

    // 2c. KVM/윈도우에 '리포트 ID가 없는' '완벽한 통합 버퍼'를 전송합니다.
    tud_hid_report(0, kvm_combined_report, KVM_REPORT_SIZE);

    // 2d. 마우스 '움직임' 데이터는 1회성이므로 전송 후 즉시 초기화합니다.
    //     (키보드 '키' 데이터는 눌린 상태 유지를 위해 초기화하지 않습니다.)
    memset(last_mouse_report, 0, 9);
    // [추가] 키보드의 마우스/소비자 제어 기능이 있다면 초기화 (안전장치)
    // (NKRO 비트맵을 제외한 나머지 부분을 0으로 클리어)
    memset(last_kb_report, 0, 1); // 모디파이어 키 초기화
}

// 3. 모니터링 리포트 전송 (이것은 원래 로직대로 둡니다)
if (monitor_enabled && tud_hid_n_ready(1)) {
    send_monitor_report(do_send_report);
}
// ---▲ KVM 호환성 v8 수정 끝 ▲---
        if (our_descriptor->main_loop_task != nullptr) {
            our_descriptor->main_loop_task();
        }
        send_out_report();
        if (need_to_persist_config) {
            persist_config_return_code = persist_config();
            need_to_persist_config = false;
        }

        print_stats_maybe();

        activity_led_off_maybe();
    }

    return 0;
}
