const uint8_t pwmpin = 3;     // PWM output
const uint8_t enpin = A7;     // A/B switch condition
const uint8_t cspin = A6;     // current detection pin
const uint8_t inApin = A4;    // A switch control pin
const uint8_t inBpin = A5;    // B switch control pin

const int buttonPin = A3;    // the number of the pushbutton pin

uint8_t buttonState = LOW;             // the current stable state
uint8_t lastReading = LOW;          // the previous reading from the input pin
uint8_t lastButtonState = LOW;   // the previous stable state
uint8_t reading = LOW;          // the current reading from the input pin

uint8_t direction = 1;      // 1 if cw, 0 if ccw
uint8_t motion = 0; // 1 if motor moves, 0 if stops
uint8_t currentThreshold = 20; // CS output threshold for the stop

uint8_t PWM = 255;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

unsigned long lastTimer1StartTime = 0; //the last time timer was set (stop if move too long)
unsigned long timer1Delay = 1000;

unsigned long lastTimer2StartTime = 0; //the last time timer was set (ignore start peak current)
unsigned long timer2Delay = 500;


void setup() {
    pinMode(buttonPin, INPUT);
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for Native USB only
    }

    pinMode(inApin, OUTPUT);
    pinMode(inBpin, OUTPUT);
    pinMode(pwmpin, OUTPUT);

}


void loop() {

    reading = digitalRead(buttonPin);


    Serial.print(" cur ");
    Serial.print(analogRead(cspin));
    Serial.print(" dir ");
    Serial.print(direction);
    Serial.print(" mot ");
    Serial.print(motion);
    Serial.print(" tim1 ");
    Serial.print((millis() - lastTimer1StartTime));
    Serial.print(" tim1del ");
    Serial.print(timer1Delay);
    Serial.print(" tim2 ");
    Serial.print((millis() - lastTimer2StartTime));
    Serial.print(" tim2del ");
    Serial.print(timer2Delay);
    Serial.print(" reading ");
    Serial.print(reading);
    Serial.print(" lastReading ");
    Serial.print(lastReading);
    Serial.print(" buttonSt ");
    Serial.print(buttonState);
    Serial.print(" lastButton ");
    Serial.print(lastButtonState);

    Serial.println("");


    if (reading != lastReading) {
        lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;
        }
    }

    if (motion) {
        if (direction) {
            if (!buttonState) {
                //smooth brake
                digitalWrite(inApin, LOW);
                digitalWrite(inBpin, LOW);
                direction = 0;
                motion = 0;
                Serial.println("button off");
            }
        }
        else {

        }
        if  ((millis() - lastTimer1StartTime) > timer1Delay) {
            // smooth brake
            digitalWrite(inApin, LOW);
            digitalWrite(inBpin, LOW);
            motion = 0;

            if (!direction) {
                direction = 1;
            }

            Serial.println("timer off");
        }
    }
    else {
        if (direction) {
            if ((buttonState != lastButtonState) && (buttonState)) {
                lastTimer1StartTime = millis();
                lastTimer2StartTime = millis();
                // clockwise rotation
                digitalWrite(inApin, HIGH);
                digitalWrite(inBpin, LOW);
                analogWrite(pwmpin, PWM);
                motion = 1;
                Serial.println("forward");
            }
            if ((buttonState != lastButtonState) && (!buttonState)) {
                direction = 0;
                lastTimer1StartTime = millis();
                lastTimer2StartTime = millis();
            }
        }
        else {
            lastTimer1StartTime = millis();
            lastTimer2StartTime = millis();
            // counterclockwise rotation
            digitalWrite(inApin, LOW);
            digitalWrite(inBpin, HIGH);
            analogWrite(pwmpin, PWM);
            motion = 1;
            Serial.println("backward");
        }
    }

    if ((analogRead(cspin) > currentThreshold) && ((millis() - lastTimer2StartTime) > timer2Delay)) {
        direction = !direction;
        //smooth brake
        digitalWrite(inApin, LOW);
        digitalWrite(inBpin, LOW);
        motion = 0;
        Serial.println("current stop");
    }

    lastReading = reading;
    lastButtonState = buttonState;
}


