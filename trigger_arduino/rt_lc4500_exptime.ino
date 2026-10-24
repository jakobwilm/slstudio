const byte ledPin = 13;

const byte trg1 = 2;
const byte trg2 = 3;

const byte trg_out = 6;

bool seqStart = true;

void setup() {

  TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
  
  pinMode(trg1, INPUT);
  pinMode(trg2, INPUT);
  attachInterrupt(digitalPinToInterrupt(trg1), isr_trg1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(trg2), isr_trg2, RISING);


  pinMode(trg_out, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(trg_out, LOW);
  digitalWrite(ledPin, LOW);
}

void loop() {
  
}

void isr_trg1() {

    // rising edge
    if (digitalRead(trg1)==HIGH){
      digitalWrite(trg_out,HIGH);
      digitalWrite(ledPin,HIGH);
    // falling edge
    }else{
      
      if(seqStart)
        delayMicroseconds(100);
      
      digitalWrite(trg_out,LOW);
      digitalWrite(ledPin,LOW);
      seqStart = false; 
    }
   
}

void isr_trg2() {
  seqStart = true;
}

