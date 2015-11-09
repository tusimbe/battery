#ifndef __KEY_H__
#define __KEY_H__

#define KEY0       (0)		 
#define KEY1       (1)		

/* key number in system */
#define KEY_NUM_IN_SYSTEM   (2)

/* key state machine flags */
#define KEY_STATE_NO_KEY    (0)
#define KEY_STATE_PRESS     (1)
#define KEY_STATE_WAITING   (2)
#define KEY_STATE_RELEASE   (3)

/* key pin state */
#define KEY_DOWN            (1)
#define KEY_UP              (2)

/* key down/up reading function protype */
typedef int16_t (*KEY_READ_FUNCPTR)(uint32_t key);

/* key state call back function */
typedef int16_t (*KEY_PRESS_CB_FUNCPTR)(uint32_t key);
typedef int16_t (*KEY_RELEASE_CB_FUNCPTR)(uint32_t key, uint32_t timer_cnt);
typedef int16_t (*KEY_WAITING_CB_FUNCPTR)(uint32_t key, uint32_t timer_cnt);

typedef struct key_conf_s
{
    uint32_t pin;
    GPIO_TypeDef *GPIO;
} KEY_CONF;


typedef struct key_info_s
{
    uint32_t valid;
    uint32_t state;
    uint32_t timer_cnt;
    uint32_t pin;
    GPIO_TypeDef *GPIO;
    KEY_READ_FUNCPTR read_func;
    KEY_PRESS_CB_FUNCPTR press_cb_func;
    KEY_WAITING_CB_FUNCPTR waiting_cb_func;
    KEY_RELEASE_CB_FUNCPTR release_cb_func;
} KEY_INFO;


void key_init(KEY_CONF key_conf[], uint32_t num_of_key);
int16_t key_scan(void);
#endif