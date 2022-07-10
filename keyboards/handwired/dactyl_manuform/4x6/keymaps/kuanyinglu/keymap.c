#include QMK_KEYBOARD_H

#define _BASE 0
#define _RAISE 1
#define _LOWER 2
#define _PLUS 3

#define SFT_TAB  SFT_T(KC_TAB)

#define KC_CAD	LALT(LCTL(KC_DEL))
#define KC_CAH	LALT(LCTL(KC_HOME))
#define KC_CAT	LALT(LCTL(KC_TAB))
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
                         KC_LBRC, KC_RBRC,  SFT_TAB, KC_SPC,                KC_LEFT, KC_RSFT, KC_LPRN, KC_RPRN,
                         KC_LCTL, KC_LOPT,  LOWER,   RAISE,                 PLUS,    KC_BSPC, KC_LCMD, KC_ENT 
    ),

    [_LOWER] = LAYOUT(
        RESET,   KC_F1,   KC_F2,   KC_F3,   KC_F4, KC_VOLU,               KC_BRIU, KC_7,    KC_8,    KC_9,    KC_PMNS, KC_PSLS,
        _______, KC_F5,   KC_F6,   KC_F7,   KC_F8, KC_VOLD,               KC_BRID, KC_4,    KC_5,    KC_6,    KC_PPLS, KC_PAST,
        _______, KC_F9,   KC_F10,  KC_F11,  KC_F12, _______,              _______, KC_1,    KC_2,    KC_3,    KC_ENT , KC_PERC,
                          _______, _______, _______, _______,             _______, _______,  KC_0,    KC_DOT,
                          _______, _______, _______, _______,             _______, KC_DEL, _______, _______
    ),

    [_RAISE] = LAYOUT(
        _______, KC_EXLM, KC_AT,   KC_HASH, KC_DLR,  _______,             _______, KC_PGDN, KC_PGUP, _______, _______, _______,
        _______, KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, _______,             KC_WH_U, KC_DOWN, KC_UP,   KC_RGHT, _______, _______,
        _______, KC_LSFT, KC_LCTL, KC_BTN1, KC_BTN2, KC_BTN3,             KC_WH_D, KC_MS_D, KC_MS_U, KC_MS_R, KC_WH_L, KC_WH_R,
                          KC_BSPC,  KC_DEL,  _______, _______,             _______, _______, _______, _______,
                          _______, _______, _______, _______,             _______, KC_MS_L, _______, _______
    ),

    [_PLUS] = LAYOUT(
        KC_CAD,  _______, _______, _______, _______, _______,             _______, _______, _______, _______, _______, _______,
        KC_CAH,  _______, _______, _______, KC_CAT,  _______,             _______, _______, _______, _______, _______, _______,
        KC_GSS,  _______, _______, _______, _______, _______,             _______, _______, _______, _______, _______, _______,
                          _______, _______, _______, _______,             _______, _______, _______, _______,
                          _______, _______, _______, _______,             _______, _______, _______, _______
    )
};

void persistent_default_layer_set(uint16_t default_layer) {
    eeconfig_update_default_layer(default_layer);
    default_layer_set(default_layer);
}
