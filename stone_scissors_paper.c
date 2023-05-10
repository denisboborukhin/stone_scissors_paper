#include <stdint.h>
#include <stdbool.h>

//---------------
// RCC Registers
//---------------

#define REG_RCC_CR     (volatile uint32_t*)(uintptr_t)0x40021000U // Clock Control Register
#define REG_RCC_CFGR   (volatile uint32_t*)(uintptr_t)0x40021004U // PLL Configuration Register
#define REG_RCC_AHBENR (volatile uint32_t*)(uintptr_t)0x40021014U // AHB1 Peripheral Clock Enable Register
#define REG_RCC_CFGR2  (volatile uint32_t*)(uintptr_t)0x4002102CU // Clock configuration register 2

//----------------
// GPIO Registers
//----------------

#define GPIOA_MODER (volatile uint32_t*)(uintptr_t)0x48000000U // GPIO port mode register
#define GPIOA_TYPER (volatile uint32_t*)(uintptr_t)0x48000004U // GPIO port output type register
#define GPIOA_PUPDR (volatile uint32_t*)(uintptr_t)0x4800000CU // GPIO port pull-up/pull-down register
#define GPIOA_IDR   (volatile uint32_t*)(uintptr_t)0x48000010U // GPIO port input  data register
#define GPIOA_ODR   (volatile uint32_t*)(uintptr_t)0x48000014U // GPIO port output data register

#define GPIOB_MODER (volatile uint32_t*)(uintptr_t)0x48000400U // GPIO port mode register
#define GPIOB_TYPER (volatile uint32_t*)(uintptr_t)0x48000404U // GPIO port output type register
#define GPIOB_PUPDR (volatile uint32_t*)(uintptr_t)0x4800040CU // GPIO port pull-up/pull-down register
#define GPIOB_IDR   (volatile uint32_t*)(uintptr_t)0x48000410U // GPIO port input  data register
#define GPIOB_ODR   (volatile uint32_t*)(uintptr_t)0x48000414U // GPIO port output data register

#define GPIOC_MODER (volatile uint32_t*)(uintptr_t)0x48000800U // GPIO port mode register
#define GPIOC_TYPER (volatile uint32_t*)(uintptr_t)0x48000804U // GPIO port output type register

//-------------------
// 7-segment display
//-------------------

// Pin Mapping:
#define A  0x0800U
#define B  0x0080U
#define C  0x0010U
#define D  0x0004U
#define E  0x0002U
#define F  0x0400U
#define G  0x0020U
#define DP 0x0008U

#define POS0 0x0040U
#define POS1 0x0100U
#define POS2 0x0200U
#define POS3 0x1000U

static const uint32_t PINS_USED = A|B|C|D|E|F|G|DP|POS0|POS1|POS2|POS3;

// TOTALLY CORRECT digit composition:
static const uint32_t DIGITS[10] =
{
    A|B|C|D|E|F,   // 0
    B|C,           // 1
    A|B|D|E|G,     // 2
    A|B|C|D|G,     // 3
    B|C|F|G,       // 4
    A|C|D|F|G,     // 5
    A|C|D|E|F|G,   // 6
    A|B|C,         // 7
    A|B|C|D|E|F|G, // 8
    A|B|C|D|F|G    // 9
};
//dislay works without two middle fields
static const uint32_t POSITIONS[2] =
{
         POS1|POS2|POS3, // 0
    //POS0     |POS2|POS3, // 1
    //POS0|POS1     |POS3, // 2
    POS0|POS1|POS2       // 3
};

// Display state:
struct Seg7Display
{
    uint32_t display;
    uint16_t number;
};

void SEG7_set_number_quarter(struct Seg7Display* seg7, unsigned tick)
{
    uint32_t divisors[2] = {1, 1000};

    unsigned quarter = tick % 2;
    unsigned divisor = divisors[quarter];

    seg7->display = DIGITS[(seg7->number / divisor) % 10] | POSITIONS[quarter];
}

// Write changes to microcontroller:
void SEG7_push_display_state_to_mc(struct Seg7Display* seg7)
{
    uint32_t surrounding_state = ~PINS_USED & *GPIOA_ODR;
    uint32_t to_write = PINS_USED & seg7->display;

    *GPIOA_ODR = surrounding_state | to_write;
}

//-------------------
// RCC configuration
//-------------------

#define CPU_FREQENCY 48000000U // CPU frequency: 48 MHz
#define ONE_MILLISECOND CPU_FREQENCY/1000U

void board_clocking_init()
{
    // (1) Clock HSE and wait for oscillations to setup.
    *REG_RCC_CR = 0x00010000U;
    while ((*REG_RCC_CR & 0x00020000U) != 0x00020000U);

    // (2) Configure PLL:
    // PREDIV output: HSE/2 = 4 MHz
    *REG_RCC_CFGR2 |= 1U;

    // (3) Select PREDIV output as PLL input (4 MHz):
    *REG_RCC_CFGR |= 0x00010000U;

    // (4) Set PLLMUL to 12:
    // SYSCLK frequency = 48 MHz
    *REG_RCC_CFGR |= (12U-2U) << 18U;

    // (5) Enable PLL:
    *REG_RCC_CR |= 0x01000000U;
    while ((*REG_RCC_CR & 0x02000000U) != 0x02000000U);

    // (6) Configure AHB frequency to 48 MHz:
    *REG_RCC_CFGR |= 0b000U << 4U;

    // (7) Select PLL as SYSCLK source:
    *REG_RCC_CFGR |= 0b10U;
    while ((*REG_RCC_CFGR & 0xCU) != 0x8U);

    // (8) Set APB frequency to 48 MHz
    *REG_RCC_CFGR |= 0b000U << 8U;
}

void regulate_ticks()
{
    for (uint32_t i = 0; i < ONE_MILLISECOND / 3U; ++i)
    {
        // Insert NOP for power consumption:
        __asm__ volatile("nop");
    }
}

void blinked_delay()
{
    for (uint32_t i = 0; i < 5U * ONE_MILLISECOND; ++i)
    {
        // Insert NOP for power consumption:
        __asm__ volatile("nop");
    }
}

//--------------------
// GPIO configuration
//--------------------

void board_gpio_init()
{
    // (1) Configure PA1-PA12 as output:
    *REG_RCC_AHBENR |= (1U << 17U);

    // Configure mode register:
    *GPIOA_MODER |= 0x1555554U;

    // Configure type register:
    *GPIOA_TYPER = 0U;

    // (2) Configure PA0 as button:
    *GPIOA_MODER |= 0U;

    // Configure PA0 as pull-down pin:
    *GPIOA_PUPDR |= (0b10U << (2U*0U));

	// Configure PB1-PB12 as input:
    *REG_RCC_AHBENR |= (1U << 18U);

    // Configure mode register:
    *GPIOB_MODER |= 0U;

    // Configure type register:
    *GPIOB_TYPER = 0U;

    // (2) Configure PA0 as button:
    *GPIOB_MODER |= 0U;

    // Configure PB0 as pull-down pin:
    *GPIOB_PUPDR |= (0b10U << (2U*0U));
    //*GPIOB_PUPDR |= (0b10U << (2U*1U));
	
    // (1) Enable GPIOC clocking:
    *REG_RCC_AHBENR |= 0x80000U;

    // (2) Configure PC8, PC9 mode:
    *GPIOC_MODER |= 0b01U << (2*8U);
    *GPIOC_MODER |= 0b01U << (2*9U);

    // (3) Configure PC8, PC9 type:
    *GPIOC_TYPER |= 0b0U << 8U;
	*GPIOC_TYPER |= 0b0U << 9U;
}

//gpio led offset
#define GPIOC_ODR_OFFSET 0x14U

const int STONE_BOTTONE_PIN = 1;
const int SCISSORS_BUTTON_PIN = 2;
const int PAPER_BUTTON_PIN = 10;


bool is_GPIOB_IDR_active(unsigned port_num)
{
    return *GPIOB_IDR & (1U << port_num);
}

bool check_button_on_press(bool is_active, int* saturation_ptr)
{
    int saturation = *saturation_ptr;
    if (is_active) {
	    if (saturation < 5) {
		    *saturation_ptr = saturation + 1;
	    }
    }
    else {
	    *saturation_ptr = 0;
		if (saturation >= 5) {
		    return 1;
		}
	}

	return 0;
}

enum Objects {
    Stone,
	Scissors,
	Paper
};

void calculate_results (char* results, enum Objects* objects)
{
    enum Objects first = objects[0];
	enum Objects second = objects[1];

    if (first == Stone) {
		if (second == Scissors)
		    results[0] += 1;
		else if (second == Paper)
		    results[1] += 1;
	}
	else if (first == Scissors) {
		if (second == Paper)
		    results[0] += 1;
		else if (second == Stone)
		    results[1] += 1;
	}
	else {
		if (second == Stone)
		    results[0] += 1;
		else if (second == Scissors)
		    results[1] += 1;
	}
}

void set_GPIOC_ODR (unsigned num_bit, bool value)
{
    if (value)
		*(volatile uint32_t*)(uintptr_t)((unsigned)GPIOC_MODER + GPIOC_ODR_OFFSET) |= (1 << num_bit);
	else
		*(volatile uint32_t*)(uintptr_t)((unsigned)GPIOC_MODER + GPIOC_ODR_OFFSET) &= ~(1 << num_bit);
}

void reverse_GPIOC_ODR (unsigned num_bit)
{
    *(volatile uint32_t*)(uintptr_t)((unsigned)GPIOC_MODER + GPIOC_ODR_OFFSET) ^= (1 << num_bit);
}

bool is_GPIOC_ODR_bit_enable(int num_bit)
{
    return *(volatile uint32_t*)(uintptr_t)((unsigned)GPIOC_MODER + GPIOC_ODR_OFFSET) & (1 << num_bit);
}

void blinked_leds ()
{
    for (int i = 0; i != 5; ++i) {
		reverse_GPIOC_ODR(8);
		reverse_GPIOC_ODR(9);
		blinked_delay();
	}
}

int get_number_from_result(char* result)
{
    return 1000 * result[0] + result[1];
}

//------
// Main
//------

int main()
{
    board_clocking_init();

    board_gpio_init();

    // Init display rendering:
    struct Seg7Display seg7 =
    {
        .number = 0
    };

    int tick = 0;

    int stone_saturation = 0;
	bool stone_button_was_pressed = 0;
    int scissors_saturation = 0;
	bool scissors_button_was_pressed = 0;
    int paper_saturation = 0;
	bool paper_button_was_pressed = 0;
	
	int player_num = 0;

	char results[2] = {0, 0};
	enum Objects objects[2];
	bool is_first_led_enable = 0;

	while (1)
    {
        // Update button state:
        bool stone_active = is_GPIOB_IDR_active(STONE_BOTTONE_PIN);
		stone_button_was_pressed = check_button_on_press(stone_active, &stone_saturation);
        bool scissors_active = is_GPIOB_IDR_active(SCISSORS_BUTTON_PIN);
		scissors_button_was_pressed = check_button_on_press(scissors_active, &scissors_saturation);
        bool paper_active = is_GPIOB_IDR_active(PAPER_BUTTON_PIN);
		paper_button_was_pressed = check_button_on_press(paper_active, &paper_saturation);

		if (stone_button_was_pressed) {
		    objects[player_num] = Stone;
				
		    is_first_led_enable = 1;
		    player_num += 1;
		}
		else if (scissors_button_was_pressed) {
		    objects[player_num] = Scissors;

		    is_first_led_enable = 1;
		    player_num += 1;
		}
		else if (paper_button_was_pressed) {
		    objects[player_num] = Paper;
		    
			is_first_led_enable = 1;
		    player_num += 1;
		}

	    if (player_num == 2) {
		    calculate_results(results, objects);
			blinked_leds ();
			set_GPIOC_ODR(9, 0);
			is_first_led_enable = 0;
		    player_num = 0;
	    }

        // Render display state:
        SEG7_set_number_quarter(&seg7, tick);
		seg7.number = get_number_from_result(results);
        SEG7_push_display_state_to_mc(&seg7);

        // Adjust ticks every ms:
        regulate_ticks();

		if (!(tick % 100)) {
		    if (is_first_led_enable) {
				set_GPIOC_ODR(8, 1);
				reverse_GPIOC_ODR(9);
		    }
		    else {
				reverse_GPIOC_ODR(8);
			}
		}

        tick += 1;
    }
}
