#include <Gaussian.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// --------------------- INFO ABOUT EDGES ---------------------
String edgeNames[5] = {"VB1", "NA", "NA", "W11", "W10"}; //name your edges (optional)
bool isWeight[5] = {0,1,0,1,1}; //is each edge a weight (1) or a bias (0)?
// (changes depending on network structure)
// B1: VB1
// W1: NA
// B2: NA
// W2: W11
// W3: W10

// --------------------- I2C STUFF ---------------------
const int wireNum = 2;
const int nBytes = 50;
int which_ei;
String last_read;

// SCL and SDA pins (19/18) should be used

// --------------------- CONSTANTS --------------------- 

const int digipot_delay_time = 10; // before/after moving digipot clicks

int k_constrain = 128; // prevent k from going to very high conductance values
int k_init_constrain = 128; //prevent k from being initialized to very high conductance values
int kwvar = 10.; // variance for initialize gaussian on weights
int kbvar = 2.; // variance for initialize gaussian on biases

int alpha = 1; //learning rate

int reset_val = 0; // k-value that actually results in neutral network (idk why this is not 0)

int L2reg_thresh = 60; //k-value (absolute val) at which point L2 regularization turns on (push you back to 0)

// --------------------- PINS --------------------- 
// globals
const int recordDpin = 17;
const int recordRpin = 16;
const int updatepin = 15;
// const int batchRecpin = 14;
// also SCL and SDA pins (19/18) should be used

// nodes
// (changes depending on network structure)
const int VbarApins[5] = {2,4,6,8,10}; //abar, na, abar, v1bar, v1bar
const int VbarBpins[5] = {3,5,7,9,11}; //v1bar, na, na, h1bar, h0bar
// abar: 2,6
// v1bar: 3,8,10
// h0bar: 11
// h1bar: 9

// edges
const int UDpospins[5] = {28, 30, 22, 39, 35};
const int CLKpospins[5] = {27, 29, 23, 40, 36};
const int UDnegpins[5] = {24, 26, 20, 37, 33};
const int CLKnegpins[5] = {12, 25, 21, 38, 34};

// --------------------- STORING VALUES IN MEMORY ---------------------
// kvals
int kvals[5] = {0,0,0,0,0};
int kposvals[5] = {0,0,0,0,0};
int knegvals[5] = {0,0,0,0,0};

// digitized node values
int DnodeA[5] = {0,0,0,0,0};
int DnodeB[5] = {0,0,0,0,0};
int RnodeA[5] = {0,0,0,0,0};
int RnodeB[5] = {0,0,0,0,0};

// banking updates for batching
int dki[5] = {0,0,0,0,0};


void setup() {
  // put your setup code here, to run once:
  pinMode(recordDpin, INPUT);
  pinMode(recordRpin, INPUT);
  pinMode(updatepin, INPUT);

  for (int i = 0; i < 5; i++){
    pinMode(VbarApins[i], INPUT);
    pinMode(VbarBpins[i], INPUT);
    pinMode(UDpospins[i], OUTPUT);
    pinMode(CLKpospins[i], OUTPUT);
    pinMode(UDnegpins[i], OUTPUT);
    pinMode(CLKnegpins[i], OUTPUT);
  }

  Wire.begin(wireNum); 
  Serial.begin(250000);
  Serial.println("starting...");
  attachInterrupt(digitalPinToInterrupt(recordDpin), recordD, RISING);
  attachInterrupt(digitalPinToInterrupt(recordRpin), recordR, RISING);
  attachInterrupt(digitalPinToInterrupt(updatepin), update, RISING);
  // Wire.onReceive(initialize_norm);
  Wire.onRequest(request_event);  
  Wire.onReceive(receive_event); 
  reset_to_0();

  print_self_info();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    String message = Serial.readStringUntil(';');

    if (message == "ps"){
      print_self_info();
    }

    if (message == "pmv"){
      int edgeselect = Serial.readStringUntil(';').toInt();
      int val = Serial.readStringUntil(';').toInt();
      int UDpin = UDpospins[edgeselect];
      int CLKpin = CLKpospins[edgeselect];
      moveDigipot(val, UDpin, CLKpin);
      Serial.print("UDpin: ");
      Serial.print(UDpin);
      Serial.print(", CLKpin: ");
      Serial.print(CLKpin);
      Serial.print(", val: ");
      Serial.println(val);
    }

    if (message == "nmv"){
      int edgeselect = Serial.readStringUntil(';').toInt();
      int val = Serial.readStringUntil(';').toInt();
      int UDpin = UDnegpins[edgeselect];
      int CLKpin = CLKnegpins[edgeselect];
      moveDigipot(val, UDpin, CLKpin);
      Serial.print("UDpin: ");
      Serial.print(UDpin);
      Serial.print(", CLKpin: ");
      Serial.print(CLKpin);
      Serial.print(", val: ");
      Serial.println(val);
    }

    if (message == "mv"){
      int edgeselect = Serial.readStringUntil(';').toInt();
      int val = Serial.readStringUntil(';').toInt();
      Serial.print("Edge ");
      Serial.print(edgeselect);
      Serial.print(", k = ");
      Serial.println(kvals[edgeselect]);
      moveEdge(edgeselect, val);
      Serial.print("new k:");
      Serial.println(kvals[edgeselect]);
    }

    if (message == "reset"){
      Serial.println("reset all");
      reset_to_val(reset_val);
    }

    if (message == "mes"){
      for (int i = 0; i < 5; i++){
        Serial.print("Edge ");
        Serial.println(i);
        Serial.print("ki = ");
        Serial.println(kvals[i]);
        Serial.print("DA = ");
        Serial.println(DnodeA[i]);
        Serial.print("DB = ");
        Serial.println(DnodeB[i]);
        Serial.print("RA = ");
        Serial.println(RnodeA[i]);
        Serial.print("RB = ");
        Serial.println(RnodeB[i]);
      }
    }

    if (message == "norm"){
      Serial.println("normalize all");
      // initialize_norm();
      initialize_uni();
      for (int i = 0; i < 5; i++){
        Serial.print("Edge ");
        Serial.print(i);
        Serial.print(", k = ");
        Serial.println(kvals[i]);
      }
    }
  }
}

void reset_to_0(){
  for (int i = 0; i < 5; i++){
    moveDigipot(-129, UDpospins[i], CLKpospins[i]);
    moveDigipot(-129, UDnegpins[i], CLKnegpins[i]);

    kvals[i] = 0;
    kposvals[i] = 0;
    knegvals[i] = 0;
  }
}

void reset_to_val(int val){
  reset_to_0();
  for (int i = 0; i < 5; i++){
    moveEdge(i, val);
  }
}

void initialize_norm(){
  Gaussian gw(reset_val, kwvar);
  Gaussian gb(reset_val, kbvar);

  reset_to_0();

  for (int i = 0; i < 5; i++){
    bool isW = isWeight[i];
    double randval;

    if (isW){
      randval = gw.random();
    }
    else{
      randval = gb.random();
    }

    int kval = int(round(randval));
    kval = constrain(kval, -1*k_init_constrain, k_init_constrain);

    moveEdge(i, kval);
  }
}

void initialize_uni(){
  reset_to_0();

  for (int i = 0; i < 5; i++){
    bool isW = isWeight[i];
    double randval;

    if (isW){
      randval = random(-1*kwvar, kwvar);
    }
    else{
      randval = random(-1*kbvar, kbvar);
    }

    int kval = int(round(randval));
    kval = constrain(kval, -1*k_init_constrain, k_init_constrain);

    moveEdge(i, kval);
  }
}

void update(){
  for (int ei = 0; ei < 5; ei++){

    // learning rule
    int dk = alpha*((DnodeA[ei]*DnodeB[ei]) - (RnodeA[ei]*RnodeB[ei]));

    moveEdge(ei, dk);
    delayMicroseconds(100);
  }
}

void storeUpdate(){
  for (int ei = 0; ei < 5; ei++){

    // learning rule
    int dk = alpha*((DnodeA[ei]*DnodeB[ei]) - (RnodeA[ei]*RnodeB[ei]));

    dki[ei] += dk;
  }
}

void batchUpdate(){
  for (int ei = 0; ei < 5; ei++){
    moveEdge(ei, dki[ei]);
    delayMicroseconds(100);

    dki[ei] = 0;
  }
}

void recordD(){
  for (int ei = 0; ei < 5; ei++){
    DnodeA[ei] = digitalRead(VbarApins[ei]);
    DnodeB[ei] = digitalRead(VbarBpins[ei]);
  }
}

void recordR(){
  for (int ei = 0; ei < 5; ei++){
    RnodeA[ei] = digitalRead(VbarApins[ei]);
    RnodeB[ei] = digitalRead(VbarBpins[ei]);
  }
}

void moveEdge(int ei, int delta){

  int currentk = kvals[ei];
  int poswiper = kposvals[ei];
  int negwiper = knegvals[ei];

  int newk = constrain(currentk + delta, -1*k_constrain, k_constrain);

  int posk = constrain(newk, 0, k_constrain);
  int negk = constrain(-1*newk, 0, k_constrain);

  int pwdelta = posk - poswiper;
  int nwdelta = negk - negwiper;

  moveDigipot(pwdelta, UDpospins[ei], CLKpospins[ei]);
  moveDigipot(nwdelta, UDnegpins[ei], CLKnegpins[ei]);

  kvals[ei] = newk;
  kposvals[ei] = posk;
  knegvals[ei] = negk;
}

// Moves digipot by delta
void moveDigipot(int delta, int UDpin, int TICKpin) {
  if (delta >= 0) {
    digitalWrite(UDpin, HIGH);  // move up
  } else {
    digitalWrite(UDpin, LOW);  // move down
    delta = -delta;
  }

  for (int d = 0; d < delta; d++) {  // cycle clock desired number of times
    delayMicroseconds(digipot_delay_time);
    digitalWrite(TICKpin, HIGH);
    delayMicroseconds(digipot_delay_time);
    digitalWrite(TICKpin, LOW);
  }
}

void print_self_info(){
  Serial.print("Wire ");
  Serial.println(wireNum);
  for (int i =0; i < 5; i++){
    Serial.print("Edge ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(edgeNames[i]);
  }
}

void request_event(){
  // this is overkill but works
  // first byte: 1/0 for positive/negative
  // second and third bytes: represent 16-bit positive number (abs(k))
  byte sendArray[3];

  if (last_read == "k"){
    int thisK =abs(kvals[which_ei]);
    bool isKpos = (kvals[which_ei] > 0);
    sendArray[0] = isKpos;
    sendArray[1] = (thisK >> 8) & 0xFF;
    sendArray[2] = thisK & 0xFF;
  }
  if (last_read == "a"){
    sendArray[0] = 0;
    sendArray[1] = DnodeA[which_ei];
    sendArray[2] = RnodeA[which_ei];
  }
  if (last_read == "b"){
    sendArray[0] = 0;
    sendArray[1] = DnodeB[which_ei];
    sendArray[2] = RnodeB[which_ei];
  }
  Serial.print(sendArray[0]);
  Serial.print(sendArray[1]);
  Serial.println(sendArray[2]);
  Wire.write(sendArray, 3);
}

void receive_event(int howMany){
  byte message = Wire.read();
  if (message == 'K'){ //change k index to be sent over next
    last_read = "k";
    byte ei = Wire.read();
    which_ei = ei;
    Serial.print("which_ei: ");
    Serial.println(which_ei);
  }
  if (message == 'A'){ //change index to be sent over next
    last_read = "a";
    byte ei = Wire.read();
    which_ei = ei;
    Serial.print("which_ei: ");
    Serial.println(which_ei);
  }
  if (message == 'B'){ //change index to be sent over next
    last_read = "b";
    byte ei = Wire.read();
    which_ei = ei;
    Serial.print("which_ei: ");
    Serial.println(which_ei);
  }
  if (message == 'R'){
    reset_to_val(reset_val);
    Serial.println("reset all");
  }
  if (message == 'N'){
    // initialize_norm();
    initialize_uni();
    Serial.println("normed");
  }
  if (message == 'U'){
    byte ei = Wire.read();
    moveEdge(ei, 20);
  }
  if (message == 'D'){
    byte ei = Wire.read();
    moveEdge(ei, -20);
  }
  if (message == 'V'){
    int val = Wire.read();
    if (val > 127) {
		val = 256 - val;
		val *= -1;
	  }
    reset_val = val;
    Serial.print("Reset val: ");
    Serial.println(reset_val);
  }
  if (message == 'W'){
    int val = Wire.read();
    kwvar = val;
    Serial.print("kwvar: ");
    Serial.println(kwvar);
  }
  if (message == 'Q'){
    int val = Wire.read();
    kbvar = val;
    Serial.print("kbvar: ");
    Serial.println(kbvar);
  }
  if (message == 'S'){
    int val = Wire.read();
    randomSeed(val);
    Serial.print("seed: ");
    Serial.println(val);
  }
}