class StepperMotor {
  private:
    const unsigned long pin_access_time_delta_millis = 1; // according video should work fine with 1millis
    unsigned long last_step_pin_access_millis = 0; // according datasheet timer will overflow  50 day;
    unsigned long target_angle_change_millis = 0;
    long int current_angle_as_step = 0, target_angle_as_step = 0;
    int step_incr_dir = -1, step_pin_status = LOW;
    int steps_per_revolution = -1;
    // void do_a_step();

  public:
    float step_angle = -1, target_angle = 0.0;
    int dir_pin = -1, step_pin = -1;
    // -1 means must be changed

    // void setup_pins(int dirPin, int stepPin);
    // void reset();
    // void do_loop();
    // void rotate(float angle);
    // float get_angle();

  public:
    StepperMotor(int dirPin, int stepPin) {
      this->setup_pins(dirPin, stepPin);
      this->steps_per_revolution = 200; // Default value from datasheet
      this->reset();
    }

    StepperMotor(int dirPin, int stepPin, int step_per_revolut) {
      this->setup_pins(dirPin, stepPin);
      this->steps_per_revolution = step_per_revolut;
      this->reset();
    }

    void reset() {
      this->current_angle_as_step = 0;
      this->step_angle = 360.0 / this->steps_per_revolution;
      digitalWrite(this->dir_pin, LOW);
      digitalWrite(this->step_pin, LOW);
      this->step_pin_status = LOW;
      this->step_incr_dir = 1;
    }

    void setup_pins(int dirPin, int stepPin) {
      this->dir_pin = dirPin;
      this->step_pin = stepPin;
      pinMode(this->dir_pin, OUTPUT);
      pinMode(this->step_pin, OUTPUT);
    }

    void do_loop() {
      if (this->target_angle_as_step != this->current_angle_as_step) {
        unsigned long current_millis = millis();
        if (current_millis >= (this->last_step_pin_access_millis + this->pin_access_time_delta_millis)) {
          // do a half step
          if (this->step_pin_status == LOW) {
            this->step_pin_status = HIGH;
          } else {
            this->step_pin_status = LOW;
            this->current_angle_as_step += this->step_incr_dir;
          }
          digitalWrite(this->step_pin, this->step_pin_status);
          this->last_step_pin_access_millis = current_millis;
        }
      }
    }

    void set_rotation(float abs_angle) {
      this->target_angle = abs_angle;

      this->target_angle_change_millis = millis();
      this->target_angle_as_step = this->target_angle / this->step_angle;
      if (this->target_angle_as_step >= this->current_angle_as_step) {
        this->step_incr_dir = 1;
        digitalWrite(this->dir_pin, HIGH);
      } else {
        this->step_incr_dir = -1;
        digitalWrite(this->dir_pin, LOW);
      }
    }

    // misc

    float get_angle() {
      return this->target_angle;
    }

    void rotate(float angle) {
      this->set_rotation(this->get_angle() + angle);
    }
};

class EightSegmentDisplayDigit {
  public:
    static const int pins_max_count = 8, symbols_map_len = 21;
    const char symb_map_keys[EightSegmentDisplayDigit::symbols_map_len] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.', 'A', 'C', 'E', 'F', 'H', 'L', 'P', 'Y', ' ', '?'};
    const byte symbs_map[EightSegmentDisplayDigit::symbols_map_len] = {
      0b11111100, // 0 or O
      0b01100000, // 1
      0b11011010, // 2
      0b11110010, // 3
      0b01100110, // 4
      0b10110110, // 5 or S
      0b10111110, // 6
      0b11100000, // 7
      0b11111110, // 8
      0b11110110, // 9
      0b00000001, // .
      0b11101110, // A
      0b10011100, // C
      0b10011110, // E
      0b10001110, // F
      0b01101110, // H
      0b00011100, // L
      0b11001110, // P
      0b01001110, // Y
      0b00000000, // empty
      0b10000101  // not found 
    };
    static constexpr int char_not_found_symb_i = EightSegmentDisplayDigit::symbols_map_len - 1;

  public:
    //                                              { a,  b,  c,  d,  e,  f,  g, dp}
    int pins[EightSegmentDisplayDigit::pins_max_count] = {-1, -1, -1, -1, -1, -1, -1, -1};
    int common_pin = -1, provided_pins_count = 0; // -1 means value must be initialised
    bool is_ca = true; // Common Anode

    EightSegmentDisplayDigit(int pins[], int pins_arr_len, int pin_common, bool is_common_catode) {
      this->provided_pins_count = min(pins_arr_len, EightSegmentDisplayDigit::pins_max_count);
      for (int i = 0; i < this->provided_pins_count; i++){
        this->pins[i] = pins[i];
      }
      this->common_pin = pin_common;
      this->is_ca = !is_common_catode;

      this->setup_pins();
      this->turn_off();
    }

    void setup_pins() {
      for (int i = 0; i < this->provided_pins_count; i++){
        pinMode(this->pins[i], OUTPUT);
      }
      pinMode(this->common_pin, OUTPUT);
    }

    void turn_off(){
      this->clear();
      digitalWrite(this->common_pin, !this->is_ca);
    }

    void clear(bool means_turn_all_on = false) {
      this->set_leds_by_int(means_turn_all_on ? ~(0b00000000) : 0b00000000);
    }

    void set_leds_by_int(byte values, byte dot_value){
      this->set_leds_by_int(values & 0b11111110 + dot_value);
    }
    void set_leds_by_int(byte values){
      digitalWrite(this->common_pin, this->is_ca);
      for (int i = 0; i < this->provided_pins_count; i++)
        digitalWrite(pins[i], (((values << i) & 0b10000000) ^ this->is_ca));
    }

    void set_leds_array(bool values[], int values_len) {
      digitalWrite(this->common_pin, this->is_ca);
      for (int i = 0; i < this->provided_pins_count; i++)
        digitalWrite(this->pins[i], (values[i] ^ this->is_ca));
    }

    void show_symbol(char symb){
      int index = EightSegmentDisplayDigit::char_not_found_symb_i;
      for(int i = 0; i < EightSegmentDisplayDigit::symbols_map_len; i++){
        if(symb == EightSegmentDisplayDigit::symb_map_keys[i]){
          index = i;
          break;
        }
      }
      this->set_leds_by_int(symbs_map[index]);
    }
};

class EightSegment4DigitDisplay{
  public:
    EightSegmentDisplayDigit** digits;
    char* text = new char[4];
    byte digits_count = 0;
    long int last_digit_update_millis = 0;
    long int digit_show_duration_ms = 1;
    byte curr_digit_i = 0;
    byte curr_text_symb_i = 0;

    EightSegment4DigitDisplay(int pins[], byte pins_count, int pins_common[], byte pins_common_count, bool is_common_catode){
      this->create_digit_objs(pins, pins_count, pins_common, pins_common_count, is_common_catode);
    }

    void create_digit_objs(int pins[], byte pins_count, int pins_common[], byte pins_common_count, bool is_common_catode){
      this->digits_count = pins_common_count;
      this->digits = (EightSegmentDisplayDigit**) malloc(sizeof(EightSegmentDisplayDigit*) * this->digits_count);

      this->text = new char[this->digits_count];
      for(byte i = 0; i < this->digits_count; i++){
        this->digits[i] = new EightSegmentDisplayDigit(pins, pins_count, pins_common[i], is_common_catode);
        this->text[i] = '.';
      }
    }

    void do_loop(){
      if((millis() - this->last_digit_update_millis) >= this->digit_show_duration_ms){
        this->digits[this->curr_digit_i]->turn_off();
        this->curr_digit_i++;
        this->curr_digit_i %= this->digits_count;
        this->last_digit_update_millis = millis();
      }
      this->digits[this->curr_digit_i]->show_symbol(this->text[this->curr_digit_i]);
    }

    void set_text(const char* new_text, byte char_count, bool align_left=true, bool clear_empty = true){
      byte min_count = min(this->digits_count, char_count);
      byte i = align_left ? 0 : (this->digits_count - min_count);
      if(clear_empty){
        for(byte k = 0; k < i; k++)
          this->text[k] = ' ';
      }

      for(byte j=0; j < min_count; i++, j++)
        this->text[i] = new_text[j];

      if(clear_empty){
        for(; i < this->digits_count; i++)
          this->text[i] = ' ';
      }
      
      // Serial.print("Text:");
      // Serial.print(this->text[0]);
      // Serial.print(this->text[1]);
      // Serial.print(this->text[2]);
      // Serial.print(this->text[3]);
      // Serial.println(";");
    }
};

#define BTN_DECR_PIN 2
#define BTN_INCR_PIN 3
#define BTN_ENTR_PIN 4

#define STEPPER_STEP_PIN 12
#define STEPPER_DIR_PIN 11

#define SEG_E A5
#define SEG_D A4
#define SEG_DP A3
#define SEG_C A2
#define SEG_G A1
#define SEG_DIG_4 A0

#define SEG_B 10
#define SEG_DIG_3 9
#define SEG_DIG_2 8
#define SEG_F 7
#define SEG_A 6
#define SEG_DIG_1 5

StepperMotor* motor;
EightSegment4DigitDisplay* display;

void setup() {
  motor = new StepperMotor(STEPPER_DIR_PIN, STEPPER_STEP_PIN);
  display = new EightSegment4DigitDisplay(
    new int[8]{SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_DP}, 8,
    new int[4]{SEG_DIG_1, SEG_DIG_2, SEG_DIG_3, SEG_DIG_4}, 4,
    true
  );
  pinMode(BTN_DECR_PIN, INPUT_PULLUP);
  pinMode(BTN_INCR_PIN, INPUT_PULLUP);
  pinMode(BTN_ENTR_PIN, INPUT_PULLUP);

  Serial.begin(115200);
}


bool btn_entr_flag = false, btn_incr_flag = false, btn_decr_flag = false;
int tooth_count = 1;
unsigned long last_read_millis = 0, curr_millis = 0;

void loop() {
  motor->do_loop();
  display->do_loop();

  curr_millis = millis();

  if (curr_millis >= (last_read_millis + 20)) {
    if (digitalRead(BTN_ENTR_PIN) == LOW)  {
      if(!btn_entr_flag){
        btn_entr_flag = true;
        float angle = 360.0 / tooth_count;

        motor->rotate(angle);

        Serial.print("rotate_by:");
        Serial.println(angle);
      }
    } else {
      btn_entr_flag = false;
    }

    if (digitalRead(BTN_INCR_PIN) == LOW) {
      if(!btn_incr_flag){
        btn_incr_flag = true;
        tooth_count++;
        if(tooth_count >= 99)
          tooth_count = 1;

        Serial.print("incr();\t tooth_count:");
        Serial.println(tooth_count);
      }
    } else {
      btn_incr_flag = false;
    }

    if (digitalRead(BTN_DECR_PIN) == LOW)  {
      if(!btn_decr_flag){
        btn_decr_flag = true;
        tooth_count--;
        if(tooth_count <= 0)
          tooth_count = 99;

        Serial.print("decr();\t tooth_count:");
        Serial.println(tooth_count);
      }
    } else {
      btn_decr_flag = false;
    }

    String a = "C." + String(tooth_count);
    display->set_text(a.c_str(), a.length());

    last_read_millis = curr_millis;
  }
}
