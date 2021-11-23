#include QMK_KEYBOARD_H

#define _BASE 0
#define _RAISE 1
#define _LOWER 2
#define _PLUS 3

#define SFT_ESC  SFT_T(KC_ESC)
#define CTL_BSPC CTL_T(KC_BSPC)
#define ALT_SPC  ALT_T(KC_SPC)
#define SFT_ENT  SFT_T(KC_ENT)

#define KC_CAD	LALT(LCTL(KC_DEL))
#define KC_CAH	LALT(LCTL(KC_HOME))
#define KC_GSS	LGUI(LSFT(KC_S))

#define RAISE MO(_RAISE)
#define LOWER MO(_LOWER)
#define PLUS MO(_PLUS)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    /* Base (qwerty)
     * +-----------------------------------------+                             +-----------------------------------------+
     * | ESC  |   q  |   w  |   e  |   r  |   t  |                             |   y  |   u  |   i  |   o  |   p  |      |
     * |------+------+------+------+------+------|                             |------+------+------+------+------+------|
     * | TAB  |   a  |   s  |   d  |   f  |   g  |                             |   h  |   j  |   k  |   l  |   ;  |      |
     * |------+------+------+------+------+------|                             |------+------+------+------+------+------|
     * | SHFT |   z  |   x  |   c  |   v  |   b  |                             |   n  |   m  |   ,  |   .  |   /  |      |
     * +------+------+------+------+-------------+                             +-------------+------+------+------+------+
     *               |  [   |   ]  |      |      |                             |      |      |      |      |
     *               +-------------+-------------+                             +-------------+-------------+
     *               |  [   |   ]  |      |      |                             |      |      |      |      |
     *               +-------------+-------------+                             +-------------+-------------+
     */
    [_BASE] = LAYOUT(
        KC_EQL,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,                  KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,   KC_GRV,         
        KC_ESC,  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,                  KC_H,    KC_J,    KC_K,    KC_L, KC_SCOLON, KC_QUOTE,         
        KC_MINS, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,                  KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH,KC_BSLS,         
                         KC_LBRC, KC_RBRC,  KC_TAB,  KC_SPC,                KC_LEFT, KC_BSPC, KC_LPRN, KC_RPRN,
                         KC_LCTL, KC_LOPT,  LOWER,   RAISE,                 PLUS,    KC_ENT,  KC_LCMD, KC_RSFT 
    ),

    [_LOWER] = LAYOUT(
        RESET,   KC_F1,   KC_F2,   KC_F3,   KC_F4, _______,               _______, KC_7,    KC_8,    KC_9,    KC_PMNS, KC_PSLS,
        _______, KC_F5,   KC_F6,   KC_F7,   KC_F8, _______,               _______, KC_4,    KC_5,    KC_6,    KC_PPLS, KC_PAST,
        _______, KC_F9,   KC_F10,  KC_F11,  KC_F12, _______,              _______, KC_1,    KC_2,    KC_3,    KC_ENT , KC_PERC,
                          _______, _______, _______, _______,             _______, KC_DEL,  KC_0,    KC_DOT,
                          _______, _______, _______, _______,             _______, _______, _______, _______
    ),

    [_RAISE] = LAYOUT(
        _______, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  _______,             KC_WH_L, KC_WH_D, KC_WH_U, KC_WH_R, _______, _______,
        _______, KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, _______,             KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, _______, _______,
        _______, _______, _______, KC_BTN1, KC_BTN2, KC_BTN3,             KC_MS_L, KC_MS_D, KC_MS_U, KC_MS_R, _______, _______,
                          _______, _______, _______, _______,             _______, KC_WH_L, _______, _______,
                          _______, _______, _______, _______,             _______, _______, _______, KC_MS_L
    ),

    [_PLUS] = LAYOUT(
        KC_CAD,  _______, _______, _______, _______, _______,             _______, _______, _______, _______, _______, _______,
        KC_CAH,  _______, _______, _______, _______, _______,             _______, _______, _______, _______, _______, _______,
        KC_GSS,  _______, _______, _______, _______, _______,             _______, _______, _______, _______, _______, _______,
                          _______, _______, _______, _______,             _______, _______, _______, _______,
                          _______, _______, _______, _______,             _______, _______, _______, _______
    )
};

void persistent_default_layer_set(uint16_t default_layer) {
    eeconfig_update_default_layer(default_layer);
    default_layer_set(default_layer);
}