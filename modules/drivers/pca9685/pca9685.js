/*!
 *  @file pca9685.js
 *
 *  @mainpage Adafruit 16-channel PWM & Servo driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the 16-channel PWM & Servo driver.
 *
 *  Designed specifically to work with the Adafruit PWM & Servo driver.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/815
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  This is Moddable version.
 *  Based on https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/0fb066c/Adafruit_PWMServoDriver.cpp
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *  SASAKI TAKERU (sasaki.takeru@gmail.com)
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

import I2C from 'pins/i2c';
import Timer from 'timer';

// REGISTER ADDRESSES
const PCA9685_MODE1 = 0x00;      /**< Mode Register 1 */
const PCA9685_MODE2 = 0x01;      /**< Mode Register 2 */
const PCA9685_SUBADR1 = 0x02;    /**< I2C-bus subaddress 1 */
const PCA9685_SUBADR2 = 0x03;    /**< I2C-bus subaddress 2 */
const PCA9685_SUBADR3 = 0x04;    /**< I2C-bus subaddress 3 */
const PCA9685_ALLCALLADR = 0x05; /**< LED All Call I2C-bus address */
const PCA9685_LED0_ON_L = 0x06;  /**< LED0 on tick, low byte*/
const PCA9685_LED0_ON_H = 0x07;  /**< LED0 on tick, high byte*/
const PCA9685_LED0_OFF_L = 0x08; /**< LED0 off tick, low byte */
const PCA9685_LED0_OFF_H = 0x09; /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
const PCA9685_ALLLED_ON_L = 0xFA;  /**< load all the LEDn_ON registers, low */
const PCA9685_ALLLED_ON_H = 0xFB;  /**< load all the LEDn_ON registers, high */
const PCA9685_ALLLED_OFF_L = 0xFC; /**< load all the LEDn_OFF registers, low */
const PCA9685_ALLLED_OFF_H = 0xFD; /**< load all the LEDn_OFF registers,high */
const PCA9685_PRESCALE = 0xFE;     /**< Prescaler for PWM output frequency */
const PCA9685_TESTMODE = 0xFF;     /**< defines the test mode to be entered */

// MODE1 bits
const MODE1_ALLCAL = 0x01;  /**< respond to LED All Call I2C-bus address */
const MODE1_SUB3 = 0x02;    /**< respond to I2C-bus subaddress 3 */
const MODE1_SUB2 = 0x04;    /**< respond to I2C-bus subaddress 2 */
const MODE1_SUB1 = 0x08;    /**< respond to I2C-bus subaddress 1 */
const MODE1_SLEEP = 0x10;   /**< Low power mode. Oscillator off */
const MODE1_AI = 0x20;      /**< Auto-Increment enabled */
const MODE1_EXTCLK = 0x40;  /**< Use EXTCLK pin clock */
const MODE1_RESTART = 0x80; /**< Restart enabled */
// MODE2 bits
const MODE2_OUTNE_0 = 0x01; /**< Active LOW output enable input */
const MODE2_OUTNE_1 = 0x02; /**< Active LOW output enable input - high impedience */
const MODE2_OUTDRV = 0x04; /**< totem pole structure vs open-drain */
const MODE2_OCH = 0x08;    /**< Outputs change on ACK vs STOP */
const MODE2_INVRT = 0x10;  /**< Output logic state inverted */

const PCA9685_I2C_ADDRESS = 0x40;      /**< Default PCA9685 I2C Slave Address */
const FREQUENCY_OSCILLATOR = 25000000; /**< Int. osc. frequency in datasheet */

const PCA9685_PRESCALE_MIN = 3;   /**< minimum prescale value */
const PCA9685_PRESCALE_MAX = 255; /**< maximum prescale value */

const ENABLE_DEBUG_OUTPUT = false;

export default class PCA9685 extends I2C {
  constructor(it) {
    super(it);
    this.begin(it.prescale);
  }

	/*!
	 *  @brief  Setups the I2C interface and hardware
	 *  @param  prescale
	 *          Sets External Clock (Optional)
	 */
  begin(prescale) {
    this.reset();
    if (prescale) {
      this.setExtClk(prescale);
    } else {
      // set a default frequency
      this.setPWMFreq(1000);
    }
    // set the default internal frequency
    this.setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  }

	/*!
	 *  @brief  Sends a reset command to the PCA9685 chip over I2C
	 */
  reset() {
    this.write8(PCA9685_MODE1, MODE1_RESTART);
    this.delay(10);
  }

	/*!
	 *  @brief  Puts board into sleep mode
	 */
  sleep() {
    const awake = this.read8(PCA9685_MODE1);
    const sleep = awake | MODE1_SLEEP; // set sleep bit high
    this.write8(PCA9685_MODE1, sleep);
    this.delay(5); // wait until cycle ends for sleep to be active
  }

	/*!
	 *  @brief  Wakes board from sleep
	 */
  wakeup() {
    const sleep = this.read8(PCA9685_MODE1);
    const wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
    this.write8(PCA9685_MODE1, wakeup);
  }

	/*!
	 *  @brief  Sets EXTCLK pin to use the external clock
	 *  @param  prescale
	 *          Configures the prescale value to be used by the external clock
	 */
  setExtClk(prescale) {
    const oldmode = this.read8(PCA9685_MODE1);
    const newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    this.write8(PCA9685_MODE1, newmode); // go to sleep, turn off internal oscillator

    // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
    // use the external clock.
    this.write8(PCA9685_MODE1, (newmode |= MODE1_EXTCLK));

    this.write8(PCA9685_PRESCALE, prescale); // set the prescaler

    this.delay(5);
    // clear the SLEEP bit to start
    this.write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Mode now 0x");
      Serial.println(read8(PCA9685_MODE1), HEX);
    }
  }

	/*!
	 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
	 *  @param  freq Floating point frequency that we will attempt to match
	 */
  setPWMFreq(freq) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Attempting to set freq ");
      Serial.println(freq);
    }
    // Range output modulation frequency is dependant on oscillator
    if (freq < 1)
      freq = 1;
    if (freq > 3500)
      freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    let prescaleval = ((this._oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN)
      prescaleval = PCA9685_PRESCALE_MIN;
    if (prescaleval > PCA9685_PRESCALE_MAX)
      prescaleval = PCA9685_PRESCALE_MAX;
    const prescale = prescaleval;

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Final pre-scale: ");
      Serial.println(prescale);
    }

    const oldmode = this.read8(PCA9685_MODE1);
    const newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    this.write8(PCA9685_MODE1, newmode);                             // go to sleep
    this.write8(PCA9685_PRESCALE, prescale); // set the prescaler
    this.write8(PCA9685_MODE1, oldmode);
    this.delay(5);
    // This sets the MODE1 register to turn on auto increment.
    this.write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Mode now 0x");
      Serial.println(this.read8(PCA9685_MODE1), HEX);
    }
  }

	/*!
	 *  @brief  Sets the output mode of the PCA9685 to either
	 *  open drain or push pull / totempole.
	 *  Warning: LEDs with integrated zener diodes should
	 *  only be driven in open drain mode.
	 *  @param  totempole Totempole if true, open drain if false.
	 */
  setOutputMode(totempole) {
    const oldmode = this.read8(PCA9685_MODE2);
    let newmode;
    if (totempole) {
      newmode = oldmode | MODE2_OUTDRV;
    } else {
      newmode = oldmode & ~MODE2_OUTDRV;
    }
    this.write8(PCA9685_MODE2, newmode);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Setting output mode: ");
      Serial.print(totempole ? "totempole" : "open drain");
      Serial.print(" by setting MODE2 to ");
      Serial.println(newmode);
    }
  }

	/*!
	 *  @brief  Reads set Prescale from PCA9685
	 *  @return prescale value
	 */
  readPrescale() {
    return this.read8(PCA9685_PRESCALE);
  }

	/*!
	 *  @brief  Gets the PWM output of one of the PCA9685 pins
	 *  @param  num One of the PWM output pins, from 0 to 15
	 *  @return requested PWM output value
	 */
  getPWM(num) {
    this.write(PCA9685_LED0_ON_L + 4 * num);
    return this.read(4);
  }

	/*!
	 *  @brief  Sets the PWM output of one of the PCA9685 pins
	 *  @param  num One of the PWM output pins, from 0 to 15
	 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
	 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
	 */
  setPWM(num, on, off) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Setting PWM ");
      Serial.print(num);
      Serial.print(": ");
      Serial.print(on);
      Serial.print("->");
      Serial.println(off);
    }

    this.write(
      PCA9685_LED0_ON_L + 4 * num,
      (on) & 0xFF,
      (on >> 8) & 0xFF,
      (off) & 0xFF,
      (off >> 8) & 0xFF,
    )
  }

	/*!
	 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
	 * on/off tick placement and properly handles a zero value as completely off and
	 * 4095 as completely on.  Optional invert parameter supports inverting the
	 * pulse for sinking to ground.
	 *   @param  num One of the PWM output pins, from 0 to 15
	 *   @param  val The number of ticks out of 4096 to be active, should be a value
	 * from 0 to 4095 inclusive.
	 *   @param  invert If true, inverts the output, defaults to 'false'
	 */
  setPin(num, val, invert) {
    // Clamp value between 0 and 4095 inclusive.
    val = min(val, 4095);
    if (invert) {
      if (val == 0) {
        // Special value for signal fully on.
        this.setPWM(num, 4096, 0);
      } else if (val == 4095) {
        // Special value for signal fully off.
        this.setPWM(num, 0, 4096);
      } else {
        this.setPWM(num, 0, 4095 - val);
      }
    } else {
      if (val == 4095) {
        // Special value for signal fully on.
        this.setPWM(num, 4096, 0);
      } else if (val == 0) {
        // Special value for signal fully off.
        this.setPWM(num, 0, 4096);
      } else {
        this.setPWM(num, 0, val);
      }
    }
  }

	/*!
	 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
	 * microseconds, output is not precise
	 *  @param  num One of the PWM output pins, from 0 to 15
	 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
	 */
  writeMicroseconds(num, Microseconds) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Setting PWM Via Microseconds on output");
      Serial.print(num);
      Serial.print(": ");
      Serial.print(Microseconds);
      Serial.println("->");
    }


    let pulse = Microseconds;
    let pulselength;
    pulselength = 1000000; // 1,000,000 us per second

    // Read prescale
    let prescale = this.readPrescale();

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print(prescale);
      Serial.println(" PCA9685 chip prescale");
    }

    // Calculate the pulse for PWM based on Equation 1 from the datasheet section
    // 7.3.5
    prescale += 1;
    pulselength *= prescale;
    pulselength /= this._oscillator_freq;

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print(pulselength);
      Serial.println(" us per bit");
    }

    pulse /= pulselength;

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print(pulse);
      Serial.println(" pulse for PWM");
    }

    this.setPWM(num, 0, Math.floor(pulse));
  }

	/*!
	 *  @brief  Getter for the internally tracked oscillator used for freq
	 * calculations
	 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
	 * introspect)
	 */
  getOscillatorFrequency() {
    return this._oscillator_freq;
  }

	/*!
	 *  @brief Setter for the internally tracked oscillator used for freq
	 * calculations
	 *  @param freq The frequency the PCA9685 should use for frequency calculations
	 */
  setOscillatorFrequency(freq) {
    this._oscillator_freq = freq;
  }

  /******************* Low level I2C interface */
  read8(addr) {
    this.write(addr);
    return this.read(1)[0];
  }

  write8(addr, d) {
    this.write(addr, d);
  }

  delay(ms) {
    Timer.delay(ms);
  }
}
