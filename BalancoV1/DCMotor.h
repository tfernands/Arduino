class DCMotor {
  
  byte spd = 255, pin1, pin2;
  
  public:  
  
  void Pinout(byte in1, byte in2){ // Pinout é o método para a declaração dos pinos que vão controlar o objeto motor
    pin1 = in1;
    pin2 = in2;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
  }   

  // Speed é o método que irá ser responsável por salvar a velocidade de atuação do motor
  void Speed(byte in1){ 
    spd = constrain(in1, 0, 255);
  }     
  
  // Forward é o método para fazer o motor girar para frente
  void Forward(){
    analogWrite(pin1, spd);
    digitalWrite(pin2, LOW);
  }   

  void Backward(){ // Backward é o método para fazer o motor girar para trás
    digitalWrite(pin1, LOW);
    analogWrite(pin2, spd);
  }
  
  void Stop(){ // Stop é o metodo para fazer o motor ficar parado.
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
};
