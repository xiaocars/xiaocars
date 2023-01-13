
// class ControlByGPIO {
// private:
//   Motor const& motor_;
//   int const in1_pin;
//   int const in2_pin;
//   MotorDirection & direction_;
// public:
//   ControlByGPIO(int const in1_pin, int constin2_pin, MotorDirection & direction);
//   void set_direction(MotorDirection & direction);
// };

// class ControlByPWM {
// private:
//   Motor const& motor_;
//   int const in1_pin;
//   int const in2_pin;
//   MotorDirection & direction_;
// public:
//   ControlByPWM(int in1_pin, int in2_pin, MotorDirection & direction);
//   void set_direction(MotorDirection & direction);
// };

// template <typename B>
// class DirectionControl {
// private:
//   Motor const& motor_;
//   MotorDirection direction_;
//   std::unique_ptr<B> control_by_ = nullptr;
//   Motor const& motor_;
//   virtual void set_direction() = 0;
// public:
//   DirectionControl(Motor const& motor, int in1_pin, int in2_pin)
//   : motor_(motor),
//     direction_(MotorDirection::RELEASE),
//     control_by_(std::make_unique<B>(motor, in1_pin, in2_pin, direction_))
//   {};

//   void set_direction(MotorDirection direction) {
//     control_by_.get().set_direction(direction);
//   }

// };






// // template <typename T>
// class DirectionControl {
// private:
//   int const in1_pin;
//   int const in2_pin;
//   MotorDirection direction_;

//   virtual void set_direction() = 0;
// public:
//   DirectionControl(int in1_pin, int in2_pin)
//     : in1_pin_(in1_pin),
//     in2_pin_(in2_pin)
//   {
//     direction_ = MotorDirection::RELEASE;
//   };

//   void set_direction(MotorDirection direction) {
//     direction_ = direction;
//     switch(direction_) {
//       case MotorDirection::RELEASE:
//         pwm_.setPWM(in1_pin_, in1_value.first, in1_value.second);
//         pwm_.setPWM(in2_pin_, in2_value.first, in2_value.second);

//         GPIO::output(in1_pin_, static_cast<int>(in1_signal));
//         GPIO::output(in2_pin_, static_cast<int>(in2_signal));
//         break;
//       case MotorDirection::FORWARD:
//         break;
//       case MotorDirection::BACKWARD:
//         break;
//     }
//   }

// };



// class AbstractMotorDirectionController {
// public:
//   AbstractMotorDirectionController(int in1_pin, int in2_pin);
  
//   virtual void set(MotorDirection direction) = 0;
// protected:
// int in1_pin_;
// int in2_pin_;
// };

// class PWMMotorDirectionController: public AbstractMotorDirectionController {
// public:
//   std::map<MotorDirection, std::pair<HIGH_LOW, HIGH_LOW>> _DIRECTION_TO_SIGNALS = {
//     {MotorDirection::RELEASE, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)},
//     {MotorDirection::FORWARD, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::LOW)},
//     {MotorDirection::BACKWARD, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)}
//   };
//   std::map<HIGH_LOW, std::pair<int, int>> _PWM_VALUES = {
//     {HIGH_LOW::LOW, std::make_pair(PWM_LOW, PWM_HIGH)},
//     {HIGH_LOW::HIGH, std::make_pair(PWM_HIGH, PWM_LOW)},
//   };

//   std::shared_ptr<PWM> pwm_;
  
//   PWMMotorDirectionController(int in1_pin, int in2_pin, std::shared_ptr<PWM> pwm);

//   void set(MotorDirection direction);

// };

// class GPIOMotorDirectionController: public AbstractMotorDirectionController {
// public:
//   std::map<MotorDirection, std::pair<HIGH_LOW, HIGH_LOW>> _DIRECTION_TO_SIGNALS = {
//     {MotorDirection::RELEASE, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::HIGH)},
//     {MotorDirection::FORWARD, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::LOW)},
//     {MotorDirection::BACKWARD, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)}
//   };

//   GPIOMotorDirectionController(int in1_pin, int in2_pin) 
//   : AbstractMotorDirectionController(in1_pin, in2_pin) {
//     setup();
//   };

//   void setup();

//   void set(MotorDirection direction);
  
// };

// class Motor {
// public:
//   int _K = 16;
//   std::string name_;
//   std::shared_ptr<PWM> pwm_;
//   int in1_pin_;
//   int in2_pin_;
//   int pwm_pin_;
//   MotorDirectionControl direction_;

//   std::shared_ptr<AbstractMotorDirectionController> controller_ = nullptr;

//   Motor(std::string name, std::shared_ptr<PWM> pwm, int in1_pin, int in2_pin, int pwm_pin, MotorDirectionControl control);

//   void set(MotorDirection direction, int speed = 0);

// private:
// };


// DirectionControl::DirectionControl(int const in1_pin, int const in2_pin)
//   : in1_pin_(in1_pin),
//   in2_pin_(in2_pin)
// {
//   direction_ = MotorDirection::RELEASE;
// }

// void DirectionControl::set_direction(MotorDirection direction) {
//   direction_ = direction;
//   switch(direction_) {
//     case MotorDirection::RELEASE:
//       pwm_.setPWM(in1_pin_, in1_value.first, in1_value.second);
//       pwm_.setPWM(in2_pin_, in2_value.first, in2_value.second);

//       GPIO::output(in1_pin_, static_cast<int>(in1_signal));
//       GPIO::output(in2_pin_, static_cast<int>(in2_signal));
//       set_direction(in1_pin_, constants::HIGH, constants::HIGH);
//       set_direction(in2_pin_, constants::HIGH, constants::HIGH);
//       break;
//     case MotorDirection::FORWARD:
//       set_direction(in1_pin_, constants::HIGH, constants::HIGH);
//       set_direction(in2_pin_, constants::HIGH, constants::HIGH);
//       break;
//     case MotorDirection::BACKWARD:
//       break;
//   }
// }

// void ControlByGPIO::set_direction(int const pin, int const signal1, int const signal2) {

// }
// void ControlByPWM::set_direction(int const pin, int const signal1, int const signal2) {

// }

// ControlByGPIO::ControlByGPIO(int const in1_pin, int const in2_pin)
//   : DirectionControl(in1_pin, in2_pin)
// {

// }

// ControlByPWM::ControlByPWM(int in1_pin, int in2_pin, PWM const& pwm)
//   : DirectionControl(in1_pin, in2_pin)
// {

// }

// AbstractMotorDirectionController::AbstractMotorDirectionController(int in1_pin, int in2_pin)
//   : in1_pin_(in1_pin), in2_pin_(in2_pin) {};

// PWMMotorDirectionController::PWMMotorDirectionController(int in1_pin, int in2_pin, std::shared_ptr<PWM> pwm)
//   : AbstractMotorDirectionController(in1_pin, in2_pin) {
//   pwm_ = pwm;
// };

// void PWMMotorDirectionController::set(MotorDirection direction) {
//   std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
//   auto in1_signal = signals.first;
//   auto in2_signal = signals.second;
//   auto in1_value = _PWM_VALUES[in1_signal];
//   auto in2_value = _PWM_VALUES[in2_signal];
//   pwm_.get()->setPWM(in1_pin_, in1_value.first, in1_value.second);
//   pwm_.get()->setPWM(in2_pin_, in2_value.first, in2_value.second);
// }

// void GPIOMotorDirectionController::setup() {
//   GPIO::setmode(GPIO::BOARD);
//   GPIO::setup(in1_pin_, GPIO::OUT);
//   GPIO::setup(in2_pin_, GPIO::OUT);
// }

// void GPIOMotorDirectionController::set(MotorDirection direction) {
//   std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
//   auto in1_signal = signals.first;
//   auto in2_signal = signals.second;
//   GPIO::output(in1_pin_, static_cast<int>(in1_signal));
//   GPIO::output(in2_pin_, static_cast<int>(in2_signal));

// }

// Motor::Motor(std::string name, std::shared_ptr<PWM> pwm, int in1_pin, int in2_pin, int pwm_pin, MotorDirectionControl control)
//  : name_(name),
//   pwm_(pwm),
//   in1_pin_(in1_pin),
//   in2_pin_(in2_pin),
//   pwm_pin_(pwm_pin),
//   direction_(control)
// {
//   switch (control) {
//     case MotorDirectionControl::PWM:
//       controller_ = std::make_shared<PWMMotorDirectionController>(in1_pin_, in2_pin_, pwm_);
//       break;
//     case MotorDirectionControl::GPIO:
//       controller_ = std::make_shared<GPIOMotorDirectionController>(in1_pin, in2_pin);
//       break;
//   }
// }

// void Motor::set(MotorDirection direction, int speed) {
//   speed = std::max(0, std::min(speed, 255));
//   controller_.get()->set(direction);
//   pwm_.get()->setPWM(pwm_pin_, 0, speed * _K);
// }
