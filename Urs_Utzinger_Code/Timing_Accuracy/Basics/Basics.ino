const int CLK_pin   = A2;

//this function produces a delay for *half* clock period (100ns), approaching 5MHz
//for Teensyduino works up to 180MHz clock such as in the Teensy 3.6
static inline void ultrashort_delay_100ns(){
  #if F_CPU <= 10000000
  //1 nops
  asm volatile("nop \n\t");
  #elif F_CPU <= 20000000
  //2 nops
  asm volatile("nop \n\tnop \n\t");
  #elif F_CPU <= 30000000
  //3 nops
  asm volatile("nop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 40000000
  //4 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 50000000
  //5 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 60000000
  //6 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 70000000
  //7 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 80000000
  //8 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 90000000
  //9 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 100000000
  //10 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 110000000
  //11 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 120000000
  //12 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 130000000
  //13 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 140000000
  //14 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 150000000
  //15 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 160000000
  //16 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 170000000
  //17 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #elif F_CPU <= 180000000
  //18 nops
  asm volatile("nop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\tnop \n\t");
  #endif
}

// 96 MHz clock
// no digitalWrite 240.18 nano seconds
// digitalWriteFast 261.05 nanoseconds
// digitalWrite 856.81 nanoseconds
inline void pulse_clock(int cycles){
  for(int i = 0; i < cycles; i++){
    digitalWriteFast(CLK_pin, HIGH); // takes 10.6   nano seconds
    ultrashort_delay_100ns();        // takes 120.09 nano seconds
    digitalWriteFast(CLK_pin, LOW);
    ultrashort_delay_100ns();
  }
}

inline void pulse_clock_timed(int duration_micros){
  elapsedMicros sinceStart_micros = 0;
  while (sinceStart_micros < duration_micros){
    digitalWriteFast(CLK_pin, HIGH); 
    ultrashort_delay_100ns();        
    digitalWriteFast(CLK_pin, LOW);  //
    ultrashort_delay_100ns();        //
  }
}

void setup() {

  //Set desired pins to OUTPUT
  pinMode(CLK_pin, OUTPUT);
  digitalWrite(CLK_pin, LOW);  // Start with CLK High
  
  while (!Serial && millis() < 3000) ;
  Serial.begin(Serial.baud());
  Serial.println("Testing timing for C12880MA");
}

float filtered = 0.0;
void loop() {
  elapsedMicros sinceStart_micros = 0;
  pulse_clock(1000000);
  Serial.println(sinceStart_micros);
}
