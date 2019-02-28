/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"
#include "quantum.h"

extern keymap_config_t keymap_config;

enum planck_layers {
  _QWERTY,
  _COLEMAK,
  _DVORAK,
  _LOWER,
  _RAISE,
  _GAMER,
  _ADJUST,
  _FN
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  COLEMAK,
  GAMER,
  DVORAK,
  FUNCTION,
  BACKLIT,
  EXT_GMR
};

/*macro declarations*/
enum {
  cad = 0,
  steam,
  ow,
  dscrd,
  orgn,
  chrome
};

/*tap dance declarations */
enum {
  //TD_LYR_FN = 0,
  TD_CTRL_CAPS = 0,
  TD_ALT_TAB,
  TD_FWD_BCK
};

/*tap dance definitions*/

qk_tap_dance_action_t tap_dance_actions[] = {
  //[TD_LYR_FN] = ACTION_TAP_DANCE_DUAL_ROLE(FUNCTION, _GAMER),
  [TD_CTRL_CAPS] = ACTION_TAP_DANCE_DOUBLE(KC_LCTL, KC_CAPS),
  [TD_ALT_TAB] = ACTION_TAP_DANCE_DOUBLE(KC_TAB, LGUI(KC_TAB)),
  [TD_FWD_BCK] = ACTION_TAP_DANCE_DOUBLE(KC_MNXT, KC_MPRV)
};


#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define FUNCTION MO(_FN)
#define CTLCAP TD(TD_CTRL_CAPS)
#define ALTTAB TD(TD_ALT_TAB)
#define FWDBCK TD(TD_FWD_BCK)


/*macro definitions*/
const macro_t *action_get_macro(keyrecord_t *record, uint8_t id, uint8_t opt) {
  switch(id) {
    case cad: {
      if (record->event.pressed) {
        return MACRO( D(LCTL), D(LALT), T(DEL), U(LCTL), U(LALT), END );
      }
    }
    case steam: {
      if (record->event.pressed) {
        return MACRO( D(LGUI), T(S), U(LGUI), W(150), T(S), T(T), T(E), T(A), T(M), W(200), T(ENT), END );
      }
    }
    case ow: {
      if (record->event.pressed) {
        return MACRO( D(LGUI), T(S), U(LGUI), W(150), T(O), T(V), T(E), T(R), T(W), T(A), T(T), T(C), T(H), W(200), T(ENT), END );
      }
    }
    case dscrd: {
      if (record->event.pressed) {
        return MACRO( D(LGUI), T(S), U(LGUI), W(150), T(D), T(I), T(S), T(C), T(O), T(R), T(D), W(200), T(ENT), END );
      }
    }
    case chrome: {
      if (record->event.pressed) {
        return MACRO(D(LGUI), T(S), U(LGUI), W(150), T(C), T(H), T(R), T(O), T(M), T(E), W(200), T(ENT), END);
      }
    }
    case orgn: {
      if (record->event.pressed) {
        return MACRO(D(LGUI), T(S), U(LGUI), W(150), T(O), T(R), T(G), T(I), T(N), W(200), T(ENT), END);
      }
    }
  }
  return MACRO_NONE;
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty 
 * ,-----------------------------------------------------------------------------------.
 * | Esc  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * | Tab  |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |  "   |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * | LSFT |   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | FN   |CTL/Cap| Alt | GUI  |Lower |    Space    |Raise | Left | Down |  Up  |Right |
 * `-----------------------------------------------------------------------------------'
 */
[_QWERTY] = LAYOUT_planck_grid(
    KC_ESC,   KC_Q,   KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSPC,
    ALTTAB,   KC_A,   KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
    KC_LSFT,  KC_Z,   KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_ENT ,
    FUNCTION, CTLCAP, KC_LALT, KC_LGUI, LOWER,   KC_SPC,  KC_SPC,  RAISE,   KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT
),

/* Colemak
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   F  |   P  |   G  |   J  |   L  |   U  |   Y  |   ;  | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * | Esc  |   A  |   R  |   S  |   T  |   D  |   H  |   N  |   E  |   I  |   O  |  "   |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * | Shift|   Z  |   X  |   C  |   V  |   B  |   K  |   M  |   ,  |   .  |   /  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Brite| Ctrl | Alt  | GUI  |Lower |    Space    |Raise | Left | Down |  Up  |Right |
 * `-----------------------------------------------------------------------------------'
 */
[_COLEMAK] = LAYOUT_planck_grid(
    KC_TAB,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_SCLN, KC_BSPC,
    KC_ESC,  KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,    KC_N,    KC_E,    KC_I,    KC_O,    KC_QUOT,
    KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_ENT ,
    BACKLIT, KC_LCTL, KC_LALT, KC_LGUI, LOWER,   KC_SPC,  KC_SPC,  RAISE,   KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT
),

/* Dvorak
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   "  |   ,  |   .  |   P  |   Y  |   F  |   G  |   C  |   R  |   L  | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * | Esc  |   A  |   O  |   E  |   U  |   I  |   D  |   H  |   T  |   N  |   S  |  /   |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * | Shift|   ;  |   Q  |   J  |   K  |   X  |   B  |   M  |   W  |   V  |   Z  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Brite| Ctrl | Alt  | GUI  |Lower |    Space    |Raise | Left | Down |  Up  |Right |
 * `-----------------------------------------------------------------------------------'
 */
[_DVORAK] = LAYOUT_planck_grid(
    KC_TAB,  KC_QUOT, KC_COMM, KC_DOT,  KC_P,    KC_Y,    KC_F,    KC_G,    KC_C,    KC_R,    KC_L,    KC_BSPC,
    KC_ESC,  KC_A,    KC_O,    KC_E,    KC_U,    KC_I,    KC_D,    KC_H,    KC_T,    KC_N,    KC_S,    KC_SLSH,
    KC_LSFT, KC_SCLN, KC_Q,    KC_J,    KC_K,    KC_X,    KC_B,    KC_M,    KC_W,    KC_V,    KC_Z,    KC_ENT ,
    BACKLIT, KC_LCTL, KC_LALT, KC_LGUI, LOWER,   KC_SPC,  KC_SPC,  RAISE,   KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |  Del |   !  |   @  |   #  |   $  |   %  |   ^  |   &  |   *  |   (  |   )  | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |   ~  |  F1  |  F2  |  F3  |  F4  |      |      |   _  |   +  |   {  |   }  |  |   |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |  F5  |      |      |RGB IO| RGB+ |RGB MD|      |      |      | Home | End  | Reset|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      | RGB- |      |     Play    |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_grid(
    KC_DEL,  KC_EXLM,  KC_AT,  KC_HASH, KC_DLR, KC_PERC, KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_BSPC,
    KC_TILD, KC_F1,    KC_F2,  KC_F3,   KC_F4,   _______, _______, KC_UNDS, KC_PLUS, KC_LCBR, KC_RCBR, KC_PIPE,
    KC_F5,   _______, _______, RGB_TOG, RGB_VAI, RGB_MOD, _______, _______, _______, KC_HOME, KC_END,  RESET,
    _______, _______, _______, RGB_VAD, _______, KC_MPLY, KC_MPLY, _______, _______, _______, _______, _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * | Del  |   1  |   2  |   3  |   4  |   5  |   6  |   7  |   8  |   9  |   0  | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |  `   |      |      |      |      |      |      |   -  |   =  |   [  |   ]  |  \   |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |      |      |      |      |      |      |      | Vol+ | Mute |Pg Up |Pg Dn |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |  Next/Back  |      | Vol- |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_grid(
    KC_DEL,     KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0, KC_BSPC,
    KC_GRV,  _______, _______, _______, _______, _______, _______, KC_MINS, KC_EQL,  KC_LBRC, KC_RBRC, KC_BSLS,
    _______, _______, _______, _______, _______, _______, _______, KC_VOLU, KC_MUTE, KC_PGUP, KC_PGDN, _______,
    _______, _______, _______, _______, _______, FWDBCK,  FWDBCK,  _______, KC_VOLD, _______, _______, _______
),

/* Gamer layer (http://opensteno.org)
 * ,-----------------------------------------------------------------------------------.
 * |   1  |  Tab |   Q  |   W  |   E  |   R  |   T  | Vol+ |      |   I  |      | Bksp |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |   2  | Caps |   A  |   S  |   D  |   F  |   G  | Vol- |  <<  | > || |  >>  |   D  |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |   3  | Shift|   Z  |   X  |   C  |   V  |   B  | Mute |   M  |      |   ^  |   Z  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Exit | Ctrl |  Alt |   4  |   5  |             |      |      |   <  |  \/  |  >   |
 * `-----------------------------------------------------------------------------------'
 */
[_GAMER] = LAYOUT_planck_grid(
    KC_1,    KC_TAB,  KC_Q,    KC_W,  KC_E,  KC_R,    KC_T, KC_VOLU, _______, KC_I,    _______, KC_BSPC,
    KC_2,    KC_CAPS, KC_A,    KC_S,  KC_D,  KC_F,    KC_G, KC_VOLD, KC_MPRV, KC_MPLY, KC_MNXT, KC_LBRC,
    KC_3,    KC_LSFT, KC_Z,    KC_X,  KC_C,  KC_V,    KC_B, KC_MUTE, _______, _______,  KC_UP,  KC_QUOT,
    EXT_GMR, KC_LCTL, KC_LALT, KC_4,  KC_5,  KC_SPC,  KC_SPC,  KC_M, _______, KC_LEFT, KC_DOWN, KC_RGHT
),

/* Adjust (Lower + Raise)
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|      |      |      |      |      |      |      |      |      |  Del |
 * |------+------+------+------+------+-------------+------+------+------+------+------|
 * |      |      |      |Aud on|Audoff|AGnorm|AGswap|Qwerty|Colemk|Dvorak| Gamer|      |
 * |------+------+------+------+------+------|------+------+------+------+------+------|
 * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_grid(
    _______, RESET,   DEBUG,   RGB_TOG, RGB_MOD, RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD,  RGB_VAI, RGB_VAD, KC_DEL ,
    _______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, QWERTY,  COLEMAK,  DVORAK,  GAMER,  _______,
    _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
),

/* FUNCTION ---- EDIT THIS LAYER
* ,-----------------------------------------------------------------------------------.
* | Power| F1   | F2   | F3   | F4   | F5   | F6   | F7   | F8   |Ovrwth| F10  |CTLALTDEL|
* |------+------+------+------+------+-------------+------+------+------+------+------|
* |Sleep |Origin|Steam|Discord| F14  | F15  | F16  | F17  | F18  | F19  | F20  | Gamer|
* |------+------+------+------+------+------|------+------+------+------+------+------|
* | Wake |      |      |Chrome|      |      |      |      |      |      |      | Reset|
* |------+------+------+------+------+------+------+------+------+------+------+------|
* |      |      |      |      |      |             |      |      |      |      |      |
* `-----------------------------------------------------------------------------------'
*/
[_FN] = LAYOUT_planck_grid(
  KC_PWR, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7, KC_F8, M(ow), KC_F10, M(cad),
  KC_SLEP, M(orgn), M(steam), M(dscrd), KC_F14, KC_F15, KC_F16, KC_F17, KC_F18, KC_F19, KC_F20 , GAMER,
  KC_WAKE, _______, _______, M(chrome), _______, _______, _______, _______, _______, _______, _______, RESET,
  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
)

};

#ifdef AUDIO_ENABLE
  float gamer_song[][2]     = SONG(PLOVER_SOUND);
  float gamer_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

uint32_t layer_state_set_user(uint32_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    //case GAMER:
      //if (record->event.pressed) {
        //set_single_persistent_default_layer(_GAMER);
      //}
      //return false;
     // break;
    case DVORAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_DVORAK);
      }
      return false;
      break;

    case GAMER:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          stop_all_notes();
          PLAY_SONG(gamer_song);
        #endif
        layer_off(_RAISE);
        layer_off(_LOWER);
        layer_off(_ADJUST);
        layer_on(_GAMER);
        if (!eeconfig_is_enabled()) {
            eeconfig_init();
        }
        keymap_config.raw = eeconfig_read_keymap();
        keymap_config.nkro = 1;
        eeconfig_update_keymap(keymap_config.raw);
      }
      return false;
      break;
    case EXT_GMR:
      if (record->event.pressed) {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(gamer_gb_song);
        #endif
        layer_off(_GAMER);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        register_code(KC_MS_WH_DOWN);
        unregister_code(KC_MS_WH_DOWN);
      #else
        register_code(KC_PGDN);
        unregister_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        register_code(KC_MS_WH_UP);
        unregister_code(KC_MS_WH_UP);
      #else
        register_code(KC_PGUP);
        unregister_code(KC_PGUP);
      #endif
    }
  }
}

void dip_update(uint8_t index, bool active) {
  switch (index) {
    case 0:
      if (active) {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(gamer_song);
        #endif
        layer_on(_ADJUST);
      } else {
        #ifdef AUDIO_ENABLE
          PLAY_SONG(gamer_gb_song);
        #endif
        layer_off(_ADJUST);
      }
      break;
    case 1:
      if (active) {
        muse_mode = true;
      } else {
        muse_mode = false;
        #ifdef AUDIO_ENABLE
          stop_all_notes();
        #endif
      }
   }
}

void matrix_scan_user(void) {
  #ifdef AUDIO_ENABLE
    if (muse_mode) {
      if (muse_counter == 0) {
        uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
        if (muse_note != last_muse_note) {
          stop_note(compute_freq_for_midi_note(last_muse_note));
          play_note(compute_freq_for_midi_note(muse_note), 0xF);
          last_muse_note = muse_note;
        }
      }
      muse_counter = (muse_counter + 1) % muse_tempo;
    }
  #endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
