#include <SoftwareSerial.h>

const uint8_t pwmpin = 3;     // определяем вывод для шим
const uint8_t enpin = A7;     // определяем вывод состояния ключей AB. Ключи открываются, если притянуть к 0.
const uint8_t cspin = A6;     // определяем вывод считывания тока
const uint8_t inApin = A4;    // определяем вывод управления ключом A
const uint8_t inBpin = A5;    // определяем вывод управления ключом В


void setup() {
    Serial.begin(9600);
    pinMode(inApin, OUTPUT);    // переводим вывод управления ключом A в режим "выход"
    pinMode(inBpin, OUTPUT);    // переводим вывод управления ключом B в режим "выход"
    pinMode(pwmpin, OUTPUT);    // переводим вывод управления ШИМ в режим выход
    // устанавливаем вращение мотора в одну сторону:
    digitalWrite(inApin, HIGH);
    digitalWrite(inBpin, LOW);
    // приращиваем ШИМ от 0 до 255:
    for (int i = 0; i < 256; i++){
        analogWrite(pwmpin, i);   // мотор плавно стартует
        delay(20);
    }
    // уменьшаем ШИМ от 255 до 0:
    for (int i = 255; i >= 0; i--){
        analogWrite(pwmpin, i);   // мотор плавно останавливается
        delay(20);
    }
    analogWrite(pwmpin, 127);   // устанавливаем ШИМ в значение 50%
    // устанавливаем вращение мотора в другую сторону:
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, HIGH);
    pinMode(enpin, OUTPUT);     // переводим вывод состояния ключей в режим "выход"
    digitalWrite(enpin, LOW);   // запрещаем мотору вращаться
    delay(5000);                // на пять секунд
    pinMode(enpin, INPUT);      // разрешаем мотору вращаться, переводя вывод состояния ключей в режим "вход"
    delay(5000);                // мотор вращается пять секунд
    //выводим значение тока мотора в серийный порт:
    Serial.println(analogRead(cspin));
    delay(5000);                // мотор вращается ещё пять секунд
    //резко останавливаем мотор (мотор в режиме "тормоз"):
    digitalWrite(inApin, HIGH);
    digitalWrite(inBpin, HIGH);
    delay(10000);
    // плавно останавливаем мотор (мотор в режиме "отключен"):
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);
}

void loop() {
//
}