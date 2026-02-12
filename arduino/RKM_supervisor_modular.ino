#include <Wire.h>

//  --------------------- NETWORK INFO --------------------- 
const int numV = 2; // number of visible nodes
const int numH = 2; // number of hidden nodes
const int maxNode = max(numV, numH); // maximum number of hidden/visible nodes (number of node boards)
const int numE = numV * numH + numV + numH; // number of edges

// Node clamping pins
const int clampValpins[maxNode] = {53, 51};

// Node analog measurement pins:
const int analogVpins[numV] = {A1, A2};
const int analogHpins[numH] = {A0, A3};
const int analogzeroPin = A4;

// Edge board I2C Addresses
const int numBoards = ceil(float(numE)/5.); // 5 edges per board (number of edge boards)
int ebAddress[numBoards] = {1, 2}; // I2C addresses of all boards

// Edge Information
String edgeNames[numE] = {"BV0", "BV1", "BH0", "BH1", "W00", "W01", "W10", "W11"}; // names of edges 
bool isWeight[numE] = {0,0,0,0,1,1,1,1}; // are edges weights (1) or biases (0)
int ebA[numE] = {1,2,1,1,1,1,2,2}; // board address of each edge
int ebNum[numE] = {1,0,0,2,4,3,4,3}; // index of edge within each board

// addressing location of visible nodes
// there are numV*numH A node measurements (weights) and numV B node measurements (V bias)
const int numVe = numV*numH + numV; // number of edges that connect to visible nodes
int Veidx[numVe]; // indices of edges that connect to visible nodes
int isAB[numVe]; // on which side do they connect to visible nodes?
int whichV[numVe]; // which visible node do they connect to?

// --------------------- PINS --------------------- 
// I2C: SCL and SDA pins (21/20) should be used

// testing an edge
const int testUDpin = 15;
const int testCLKpin = 14;

// edges
const int recordDpin = 42;
const int recordRpin = 46;
const int updatepin = 52;
// const int batchRecpin = 51;

// nodes (global)
const int FBpin = 39;
const int usePrevpin = 35;
const int readMempin = 27;
const int storeMempin = 25;

const int aBarpin = 53; //always output digital 1

// switch between noisy and deterministic digitization
const int detpin = 23;

// nodes (local)
// const int clampValpins[maxNode] = {53, 51};
// later this will be replaced with one clamp value and a multiplexer

// measurements
// later replace with one bar, pos, neg for each and multiplex

// --------------------- CONSTANTS --------------------- 
const int digipot_delay_time = 10; // before/after moving digipot clicks
const int trigger_delay = 1000; //for the RC delay on readmem to catch up
const int other_delay = 200; //just in case!
const int equil_delay = 1000; //for the network to equilibrate
String buffer = "        ";
String divider = ":";
int nmestimes = 5; // how many times to measure analog values

//  --------------------- TRAINING INFO --------------------- 
const int numtrain = 2; // number of training data points
int dataset[numtrain][numV] = {{1,0}, {0,1}};
const int numtest = 20; // number of test iterations (cycling through all datapoints)
int testidx[numtest];
int randomperm[numtrain];

int alpha = 1;
int measureEvery = 1;
int detInf=0; //evaluate reconstruction MSE using noisy (0) or deterministic (1) comparator

//  --------------------- VARIABLES --------------------- 
int clampVals[numV];
int Vinit[numV];
int usePrev;
int numProp = 1; //how many times to propagate in R state
byte last_rq;
int last_ei;
int last_k;
int last_DA;
int last_DB;
int last_RA;
int last_RB;

//  --------------------- STORING DATA --------------------- 
int ki[numE] = {0};
int DnodeA[numE] = {0};
int DnodeB[numE] = {0};
int RnodeA[numE] = {0};
int RnodeB[numE] = {0};

//  ------------------------------------------------------- 

void setup() {
  pinMode(recordDpin, OUTPUT);
  pinMode(recordRpin, OUTPUT);
  pinMode(updatepin, OUTPUT);

  pinMode(FBpin, OUTPUT);
  pinMode(usePrevpin, OUTPUT);
  pinMode(readMempin, OUTPUT);
  pinMode(storeMempin, OUTPUT);

  pinMode(aBarpin, OUTPUT);
  digitalWrite(aBarpin, 1);

  pinMode(detpin, OUTPUT);
  digitalWrite(detpin, 0);

  analogReadResolution(12);

  for (int i = 0; i < maxNode; i++){
    pinMode(clampValpins[i], OUTPUT);
    digitalWrite(clampValpins[i], 0);
    delayMicroseconds(other_delay);
  }

  digitalWrite(FBpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(usePrevpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(readMempin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);

  digitalWrite(recordDpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(recordRpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(updatepin, 0);
  delayMicroseconds(other_delay);

  populate_Ves();

  Wire.begin();                                                             
  Serial.begin(250000);
  Serial.println("starting...");

  // Serial.print("numBoards = ");
  // Serial.println(numBoards);

  // for (int i = 0; i < numVe; i++){
  //   Serial.print("Edge: ");
  //   Serial.print(Veidx[i]);
  //   Serial.print(", isAB = ");
  //   Serial.print(isAB[i]);
  //   Serial.print(", V node: ");
  //   Serial.println(whichV[i]);
  // }

  randomSeed(analogRead(0));
  int initval = 0;
  for (int i = 0; i < numBoards; i++){
    int add = ebAddress[i];
    int randval = random(initval, initval + 20);
    set_request(add, 'S', randval);
    Serial.print("EB: ");
    Serial.print(add);
    Serial.print(", seed: ");
    Serial.println(randval);
    initval = randval;
  }
}

void loop() {
  String message = Serial.readStringUntil(';');

  if (message == "mvtst"){
    int val = Serial.readStringUntil(';').toInt();
    moveDigipot(val, testUDpin, testCLKpin);
    sendMessageWithVarToPython("move test digi", val);
  }

  if (message == "mes"){
    full_measurement(1);
    sendMessageToPython("finished");
  }  

  if (message == "ames"){
    int isv = Serial.readStringUntil(';').toInt(); // read V or H nodes
    analog_measurement(isv, nmestimes);
    sendMessageToPython("finished");
  }

  if (message == "rq"){ //request value from slave
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    get_request(add);
    sendMessageToPython("requested");
    sendMessageToPython("finished");
  }

  if (message == "chk"){ //choose which k to request
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    int val = Serial.readStringUntil(';').toInt(); // edge index
    set_request(add, 'K', val);
    Serial.print("board = ");
    Serial.print(add);
    Serial.print("; k, i = ");
    Serial.println(val);
  }

  if (message == "cha"){ //choose which A to request
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    int val = Serial.readStringUntil(';').toInt(); // edge index
    set_request(add, 'A', val);
    Serial.print("board = ");
    Serial.print(add);
    Serial.print("; A, i = ");
    Serial.println(val);
  }

  if (message == "chb"){ //choose which B to request
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    int val = Serial.readStringUntil(';').toInt(); // edge index
    set_request(add, 'B', val);
    last_ei = val; // HELP should this be here?
    Serial.print("board = ");
    Serial.print(add);
    Serial.print("; B, i = ");
    Serial.println(val);
  }

  if (message == "mvu"){ 
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    int val = Serial.readStringUntil(';').toInt(); // edge index
    set_request(add, 'U', val);
    sendMessageWithVarToPython("board", add);
    sendMessageWithVarToPython("move edge up", val);
  }

  if (message == "mvd"){ 
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    int val = Serial.readStringUntil(';').toInt(); // edge index
    set_request(add, 'D', val);
    sendMessageWithVarToPython("board", add);
    sendMessageWithVarToPython("move edge down", val);
  }

  if (message == "reset"){
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      Wire.beginTransmission(add);  
      Wire.write('R');                                                                                             
      Wire.endTransmission();
    }
    sendMessageToPython("reset");
    sendMessageToPython("finished");
  }

  if (message == "seed"){ 
    int add = Serial.readStringUntil(';').toInt(); // address of slave
    int val = Serial.readStringUntil(';').toInt(); // edge index
    set_request(add, 'S', val);
    sendMessageWithVarToPython("board", add);
    sendMessageWithVarToPython("seed", val);
    sendMessageToPython("finished");
  }

  if (message == "norm"){ 
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      Wire.beginTransmission(add);  
      Wire.write('N');                                                                                             
      Wire.endTransmission();
    }
    sendMessageToPython("normed");
    sendMessageToPython("finished");
  }

  if (message == "resetval"){
    int val = Serial.readStringUntil(';').toInt(); // edge index
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      set_request(add, 'V', val);
    }
    sendMessageWithVarToPython("resetval", val);
    sendMessageToPython("finished");
  }

  if (message == "kwvar"){
    int val = Serial.readStringUntil(';').toInt(); // edge index
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      set_request(add, 'W', val);
    }
    sendMessageWithVarToPython("kwvar", val);
    sendMessageToPython("finished");
  }

  if (message == "kbvar"){
    int val = Serial.readStringUntil(';').toInt(); // edge index
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      set_request(add, 'Q', val);
    }
    sendMessageWithVarToPython("kbvar", val);
    sendMessageToPython("finished");
  }

  if (message == "L2"){
    int val = Serial.readStringUntil(';').toInt(); // edge index
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      set_request(add, 'L', val);
    }
    sendMessageWithVarToPython("L2", val);
    sendMessageToPython("finished");
  }

   if (message == "L2max"){
    int val = Serial.readStringUntil(';').toInt(); // edge index
    for (int i = 0; i < numBoards; i++){
      int add = ebAddress[i];
      set_request(add, 'M', val);
    }
    sendMessageWithVarToPython("L2max", val);
    sendMessageToPython("finished");
  }

  if (message == "fb"){ 
    int val = Serial.readStringUntil(';').toInt();
    digitalWrite(FBpin, val);
    // Serial.print("FB = ");
    // Serial.println(val);
    sendMessageWithVarToPython("FB", val);
    sendMessageToPython("finished");
  }

  if (message == "useprev"){ 
    int val = Serial.readStringUntil(';').toInt();
    digitalWrite(usePrevpin, val);
    // Serial.print("usePrev = ");
    // Serial.println(val);
    sendMessageWithVarToPython("useprev", val);
    sendMessageToPython("finished");
  }

  if (message == "readmem"){ 
    digitalWrite(readMempin, 1);
    delayMicroseconds(trigger_delay);
    sendMessageToPython("readMem triggered");
    digitalWrite(readMempin, 0);
    sendMessageToPython("finished");
  }

  if (message == "storemem"){ 
    digitalWrite(storeMempin, 1);
    delayMicroseconds(other_delay);
    sendMessageToPython("storeMem triggered");
    digitalWrite(storeMempin, 0);
    sendMessageToPython("finished");
  }

  if (message == "rd"){ 
    digitalWrite(recordDpin, 1);
    delayMicroseconds(other_delay);
    sendMessageToPython("recordD triggered");
    digitalWrite(recordDpin, 0);
    sendMessageToPython("finished");
  }

  if (message == "rr"){ 
    digitalWrite(recordRpin, 1);
    delayMicroseconds(other_delay);
    sendMessageToPython("recordR triggered");
    digitalWrite(recordRpin, 0);
    sendMessageToPython("finished");
  }

  if (message == "update"){ 
    for (int al = 0; al < alpha; al++){
      digitalWrite(updatepin, 1);
      delayMicroseconds(other_delay);
      sendMessageToPython("update triggered");
      digitalWrite(updatepin, 0);
      sendMessageToPython("finished");
    }
  }

  if (message == "det"){ 
    int val = Serial.readStringUntil(';').toInt();
    digitalWrite(detpin, val);
    // Serial.print("FB = ");
    // Serial.println(val);
    sendMessageWithVarToPython("det", val);
    sendMessageToPython("finished");
  }

  if (message == "detinf"){ 
    int val = Serial.readStringUntil(';').toInt();
    detInf = val;
    sendMessageWithVarToPython("detInf", val);
    sendMessageToPython("finished");
  }

  if (message == "alpha"){ 
    int alf = Serial.readStringUntil(';').toInt();
    alpha = alf;
    sendMessageWithVarToPython("alpha", alpha);
    sendMessageToPython("finished");
  }

    if (message == "measureEvery"){ 
    int mese = Serial.readStringUntil(';').toInt();
    measureEvery = mese;
    sendMessageWithVarToPython("measureEvery", measureEvery);
    sendMessageToPython("finished");
  }

  if (message == "clamp"){ 
    // rewrite this
    int localvals[maxNode] = {};
    for (int nn = 0; nn < maxNode; nn++){
      int cvn = Serial.readStringUntil(';').toInt();
      digitalWrite(clampValpins[nn],cvn);
      localvals[nn] = cvn;
    }
    send1DArrayToPython("clamped_vals", localvals, maxNode);
    sendMessageToPython("finished");
  }

  if (message == "Dstate"){ 
    // rewrite
    // clampVals[0] = 1;
    // clampVals[1] = 0;
    for (int nn = 0; nn < numV; nn++){
      int cvn = Serial.readStringUntil(';').toInt();
      clampVals[nn] = cvn;
    }
    D_state();
    sendMessageToPython("finished");
  }

  if (message == "Rstate"){ 
    // rewrite
    usePrev = 0;
    // clampVals[0] = 1;
    // clampVals[1] = 0;
    for (int nn = 0; nn < numV; nn++){
      int cvn = Serial.readStringUntil(';').toInt();
      clampVals[nn] = cvn;
    }
    R_state(1);
    sendMessageToPython("finished");
  }

  if (message == "Rstatef1"){ 
    usePrev = 0;
    for (int nn = 0; nn < numV; nn++){
      int cvn = Serial.readStringUntil(';').toInt();
      clampVals[nn] = cvn;
    }
    R_state_f1(1,1);
    sendMessageToPython("finished");
  }

  if (message == "Rstateb1"){
    usePrev = 1;
    R_state_b1(1,1);
    sendMessageToPython("finished");
  }

  if (message == "Rstatef2"){
    usePrev = 1;
    R_state_f1(1,1);
    sendMessageToPython("finished");
  }

  if (message == "train"){
    int nepochs = Serial.readStringUntil(';').toInt();
    train(nepochs);
    sendMessageToPython("finished");
  }

  if (message == "recon"){
    // rewrite
    // int V0 = Serial.readStringUntil(';').toInt();
    // // int V1 = Serial.readStringUntil(';').toInt();
    // clampVals[0] = V0;
    // // clampVals[1] = V1;
    for (int nn = 0; nn < numV; nn++){
      int cvn = Serial.readStringUntil(';').toInt();
      clampVals[nn] = cvn;
    }
    reconstruct();
    sendMessageToPython("finished");
  }

  if (message == "numprop"){
    numProp = Serial.readStringUntil(';').toInt();
    sendMessageWithVarToPython("numProp", numProp);
    sendMessageToPython("finished");
  }
}

// --------------------- FUNCTIONS --------------------- 

void train(int numepochs){

  for (int i = 0; i < numtest; i ++){
    testidx[i] = random(numtrain);  // make a test dataset first
  }

  for (int v=0; v < numV; v++){
    Vinit[v] = random(2); //initialize random Vinits
  }

  test_reconstruction(); //get initial test data reconstructions
  sendMessageToPython("finalize");

  usePrev = 0;
  int trainingstep = 0;
  int mstep = 0;
  int aRecord = 0;
  for (int epoch = 0; epoch < numepochs; epoch++){
    get_random_permutation(numtest);
    mstep++;

    if (mstep == measureEvery){
      aRecord = 1;
    }
    else{
      aRecord = 0;
    }

    for (int pi = 0; pi < numtrain; pi++){
      // fix later
      // int trainidx = orderings[perm][pi];
      int trainidx = randomperm[pi];

      for (int vi = 0; vi < numV; vi++){
        clampVals[vi] = dataset[trainidx][vi];
      }

      D_state();

      for (int vi = 0; vi < numV; vi++){
        clampVals[vi] = Vinit[vi];
      }

      sendMessageWithVarToPython("aRecord", aRecord);
      R_state(aRecord);

      // update
      for (int al = 0; al < alpha; al++){
        digitalWrite(updatepin, 1);
        delayMicroseconds(other_delay);
        sendMessageToPython("update triggered");
        digitalWrite(updatepin, 0);
        delayMicroseconds(other_delay);
      }
      trainingstep ++;
  }
  if (mstep == measureEvery){
      mstep = 0;
      sendMessageWithVarToPython("epoch", epoch);
      // sendMessageWithVarToPython("trainingstep", trainingstep);
      full_measurement(1);
      test_reconstruction();
      delayMicroseconds(100);
      sendMessageToPython("finalize");
  }
  }
}

void test_reconstruction(){
    if (detInf){
      digitalWrite(detpin, 1);
      delayMicroseconds(other_delay);
      sendMessageToPython("reconstruction_deterministic");
      for (int j = 0; j < numtest; j++){
        sendMessageWithVarToPython("testidx", j);
        clampVals[0] = dataset[testidx[j]][0];
        clampVals[1] = dataset[testidx[j]][1];
        // send1DArrayToPython("Vtest", clampVals, 2);
        reconstruct();
        // send1DArrayToPython("Vrecon", Vbar, 2);
      }
    sendMessageToPython("finalizereconstructdet");
    }
    digitalWrite(detpin, 0);
    delayMicroseconds(other_delay);
    sendMessageToPython("reconstruction");
    for (int j = 0; j < numtest; j++){
        sendMessageWithVarToPython("testidx", j);
        clampVals[0] = dataset[testidx[j]][0];
        clampVals[1] = dataset[testidx[j]][1];
        // send1DArrayToPython("Vtest", clampVals, 2);
        reconstruct();
        // send1DArrayToPython("Vrecon", Vbar, 2);
    }
    sendMessageToPython("finalizereconstruct");
}

void reconstruct(){
  // TO DO: add propagation

  digitalWrite(readMempin, 0);
  delayMicroseconds(trigger_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);

  // FB = 0 (forward)
  digitalWrite(FBpin, 0);
  delayMicroseconds(other_delay);
  // sendMessageToPython("FB = 0");

  // useprev = 0 (clamp data values)
  digitalWrite(usePrevpin, 0);
  delayMicroseconds(other_delay);
  // sendMessageToPython("useprev = 0");

  for (int i = 0; i < numV; i ++){
    digitalWrite(clampValpins[i], clampVals[i]);
    // delayMicroseconds(other_delay);
    delayMicroseconds(10000);
  }

  send1DArrayToPython("Vtest", clampVals, numV);

  delayMicroseconds(equil_delay);

  // trigger comparator outputs into D flop
  digitalWrite(readMempin, 1);
  delayMicroseconds(trigger_delay);

  // store comparator outputs in second D flop
  digitalWrite(storeMempin, 1);
  delayMicroseconds(other_delay);

  digitalWrite(readMempin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);

  // FB = 1 (backward)
  digitalWrite(FBpin, 1);
  delayMicroseconds(other_delay);

  // useprev = 1 (stored inputs)
  digitalWrite(usePrevpin, 1);
  delayMicroseconds(other_delay);

  delayMicroseconds(equil_delay);

  // trigger comparator outputs into D flop
  digitalWrite(readMempin, 1);
  delayMicroseconds(trigger_delay);

  // record digitized values in edge MCC
  digitalWrite(recordDpin, 1);
  delayMicroseconds(other_delay);

  full_measurement(0);

  int reconstructed[numV];
  for (int i = 0; i < numV; i++){
    reconstructed[i] = -1; // initialize everything to -1
  }
  for (int i = 0; i < numVe; i++){
    int AB = isAB[i];
    int Vei = Veidx[i];
    int Vi = whichV[i];
    int localmes;
    if (AB){ // B
      localmes = DnodeB[Vei];
    }
    else{ // A
      localmes = DnodeA[Vei];
    }

    if (reconstructed[Vi] == -1){
      reconstructed[Vi] = localmes; // if it's the first time you encounter this Vi then overwrite
    }
    else{
      if (reconstructed[Vi] != localmes){
        sendMessageToPython("something went wrong, measurements dont match"); //if it's not the first time, just chekc that measurements match each other
      }
    }
  }

  send1DArrayToPython("Vrecon", reconstructed, numV);

  digitalWrite(readMempin, 0);
  delayMicroseconds(trigger_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(recordDpin, 0);
}


// --------------------- RKM states --------------------- 

void D_state(){
  // reset triggers
  digitalWrite(readMempin, 0);
  delayMicroseconds(trigger_delay);
  digitalWrite(recordDpin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("all triggers reset");

  // FB = 0 (forward)
  digitalWrite(FBpin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("FB = 0");

  // useprev = 0 (clamp data values)
  digitalWrite(usePrevpin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("useprev = 0");

  // clamp values (only used if useprev = 0)
  for (int i = 0; i < numV; i ++){
    digitalWrite(clampValpins[i], clampVals[i]);
    delayMicroseconds(other_delay);
  }
 sendMessageToPython("clamped data vals");

  delayMicroseconds(equil_delay);

  // trigger comparator outputs into D flop
  digitalWrite(readMempin, 1);
  delayMicroseconds(trigger_delay);
  sendMessageToPython("readMem triggered");

  // record digitized values in edge MCC
  digitalWrite(recordDpin, 1);
  delayMicroseconds(other_delay);
  sendMessageToPython("recordD triggered");

  // reset triggers
  digitalWrite(readMempin, 0);
  delayMicroseconds(trigger_delay);
  digitalWrite(recordDpin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("all triggers reset");
}

void R_state_f1(int record, int analogRecord){

  digitalWrite(readMempin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(recordRpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("all triggers reset");

  digitalWrite(FBpin, 0);
  delayMicroseconds(other_delay);  

  digitalWrite(usePrevpin, usePrev);
  delayMicroseconds(other_delay);
  sendMessageWithVarToPython("usePrev", usePrev);

  if (!usePrev){
    // clamp values (only used if useprev = 0)
    for (int i = 0; i < 2; i ++){
      digitalWrite(clampValpins[i], clampVals[i]);
      delayMicroseconds(other_delay);
    }
  }

  delayMicroseconds(equil_delay);

  // trigger comparator outputs into D flop
  digitalWrite(readMempin, 1);
  delayMicroseconds(trigger_delay);
  sendMessageToPython("readMem triggered");

  if (analogRecord){
    // analog_measurement(1, nmestimes); //take analog measurement of V nodes
    analog_measurement(0, nmestimes); //take analog measurement of H nodes
  }

  if (record){
    // record digitized values in edge MCC
    digitalWrite(recordRpin, 1);
    delayMicroseconds(other_delay);
    sendMessageToPython("recordR triggered");
  }

  // store comparator outputs in second D flop
  digitalWrite(storeMempin, 1);
  delayMicroseconds(other_delay);
  sendMessageToPython("storeMem triggered");

  // reset triggers
  digitalWrite(readMempin, 0);
  delayMicroseconds(trigger_delay);
  digitalWrite(recordRpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("all triggers reset");
}

void R_state_b1(int record, int analogRecord){
  digitalWrite(readMempin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(recordRpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("all triggers reset");

  digitalWrite(FBpin, 1);
  delayMicroseconds(other_delay);  

  digitalWrite(usePrevpin, usePrev);
  delayMicroseconds(other_delay);
  sendMessageWithVarToPython("usePrev", usePrev);

  delayMicroseconds(equil_delay);

  // trigger comparator outputs into D flop
  digitalWrite(readMempin, 1);
  delayMicroseconds(trigger_delay);
  sendMessageToPython("readMem triggered");

  if (analogRecord){
    analog_measurement(1, nmestimes); //take analog measurement of V nodes
    // analog_measurement(0, nmestimes); //take analog measurement of H nodes
  }

  if (record){
    // record digitized values in edge MCC
    digitalWrite(recordRpin, 1);
    delayMicroseconds(other_delay);
    sendMessageToPython("recordR triggered");
  }

  // store comparator outputs in second D flop
  digitalWrite(storeMempin, 1);
  delayMicroseconds(other_delay);
  sendMessageToPython("storeMem triggered");

  // reset triggers
  digitalWrite(readMempin, 0);
  delayMicroseconds(trigger_delay);
  digitalWrite(recordRpin, 0);
  delayMicroseconds(other_delay);
  digitalWrite(storeMempin, 0);
  delayMicroseconds(other_delay);
  sendMessageToPython("all triggers reset");
}


// void R_state(){
//   // reset triggers
//   digitalWrite(readMempin, 0);
//   delayMicroseconds(trigger_delay);
//   digitalWrite(recordRpin, 0);
//   delayMicroseconds(other_delay);
//   digitalWrite(storeMempin, 0);
//   delayMicroseconds(other_delay);
//   sendMessageToPython("all triggers reset");

//   for (int i = 0; i < numProp; i++){
//     // Serial.print("prop = ");
//     // Serial.println(i);
//     sendMessageWithVarToPython("prop", i);

//     for (int j = 0; j < 2; j++){
//       // do one forward, one backward per propagation step
//       digitalWrite(FBpin, j);
//       delayMicroseconds(other_delay);
//       // Serial.print("FB = ");
//       // Serial.println(j);
//       sendMessageWithVarToPython("FB", j);

//       // set useprev
//       digitalWrite(usePrevpin, usePrev);
//       delayMicroseconds(other_delay);
//       // Serial.print("useprev = ");
//       // Serial.println(usePrev);
//       sendMessageWithVarToPython("usePrev", usePrev);

//       if (!usePrev){
//         // clamp values (only used if useprev = 0)
//         for (int i = 0; i < 2; i ++){
//           digitalWrite(clampValpins[i], clampVals[i]);
//           delayMicroseconds(other_delay);
//         }
//         // if useprev = 0 to start, switch it to 1 after the first iteration
//         usePrev = 1;
//       }

//       delayMicroseconds(equil_delay);

//       // trigger comparator outputs into D flop
//       digitalWrite(readMempin, 1);
//       delayMicroseconds(trigger_delay);
//       sendMessageToPython("readMem triggered");

//       // store comparator outputs in second D flop
//       digitalWrite(storeMempin, 1);
//       delayMicroseconds(other_delay);
//       sendMessageToPython("storeMem triggered");

//       // reset values
//       digitalWrite(readMempin, 0);
//       delayMicroseconds(trigger_delay);
//       digitalWrite(storeMempin, 0);
//       delayMicroseconds(other_delay);
//       sendMessageToPython("all triggers reset");
      
//       delay(1000);
//     }
//   }
//   analog_measurement(1, nmestimes); //take analog measurement of V nodes

//   // last step is always forward
//   digitalWrite(FBpin, 0);
//   delayMicroseconds(other_delay);
//   sendMessageToPython("FB = 0");

//   // useprev is 1
//   digitalWrite(usePrevpin, 1);
//   delayMicroseconds(other_delay);
//   sendMessageToPython("usePrev = 1");

//    // trigger comparator outputs into D flop
//   digitalWrite(readMempin, 1);
//   delayMicroseconds(trigger_delay);
//   sendMessageToPython("readMem triggered");

//   delay(1000);
//   sendMessageToPython("big delay");

//   // record digitized values in edge MCC
//   digitalWrite(recordRpin, 1);
//   delayMicroseconds(other_delay);
//   sendMessageToPython("recordR triggered");

//   // store comparator outputs in second D flop
//   digitalWrite(storeMempin, 1);
//   delayMicroseconds(other_delay);
//   sendMessageToPython("storeMem triggered");

//   // reset triggers
//   digitalWrite(readMempin, 0);
//   delayMicroseconds(trigger_delay);
//   digitalWrite(recordRpin, 0);
//   delayMicroseconds(other_delay);
//   digitalWrite(storeMempin, 0);
//   delayMicroseconds(other_delay);
//   sendMessageToPython("all triggers reset");

//   analog_measurement(0, nmestimes); // take analog measurement of H nodes
// }

void R_state(int analogRecord){
  R_state_f1(0,0);
  usePrev = 1;
  R_state_b1(0,analogRecord);
  usePrev = 1;
  R_state_f1(1,analogRecord);
}

// --------------------- Measurement --------------------- 

void full_measurement(int report){
  byte RQs[3] = {'K', 'A', 'B'};

  for (int j = 0; j < numE; j++){
    int add = ebA[j];
    int ei = ebNum[j];
    // sendMessageWithVarToPython("name", edgeNames[j]);
    // Serial.println(edgeNames[j]);
    // sendMessageWithVarToPython("board", add);
    // sendMessageWithVarToPython("index", ei);
    for (int i = 0; i < 3; i++){
      set_request(add, RQs[i], ei);
      delayMicroseconds(1000);
      // delay(1000);
      get_request(add);
      // delay(1000);

      ki[j] = last_k;
      DnodeA[j] = last_DA;
      DnodeB[j] = last_DB;
      RnodeA[j] = last_RA;
      RnodeB[j] = last_RB;
    }
  }
  if (report){
    send1DArrayToPython("ki", ki, numE);
    send1DArrayToPython("DA", DnodeA, numE);
    send1DArrayToPython("DB", DnodeB, numE);
    send1DArrayToPython("RA", RnodeA, numE);
    send1DArrayToPython("RB", RnodeB, numE);
  }
}

void analog_measurement(int isV, int ntimes){
  int readVal;
  sendMessageWithVarToPython("nmestimes", ntimes);
  readVal = 0;
  for (int nt = 0; nt < ntimes; nt++){
    readVal += analogRead(analogzeroPin);
  }
  sendMessageWithVarToPython("analogZero", readVal);
  if (isV){
    int Vvals[numV];
    for (int i = 0; i < numV; i++){
      readVal = 0;
      for (int nt = 0; nt < ntimes; nt++){
        readVal += analogRead(analogVpins[i]);
      }
      Vvals[i] = readVal;
    }
    send1DArrayToPython("analogV", Vvals, numV);
  }
  else{
    int Hvals[numH];
    for (int i = 0; i < numH; i++){
      readVal = 0;
      for (int nt = 0; nt < ntimes; nt++){
        readVal += analogRead(analogHpins[i]);
      }
      Hvals[i] = readVal;
    }
    send1DArrayToPython("analogH", Hvals, numH);
  }
}

// --------------------- I2C send/receive --------------------- 

void set_request(int address, byte rq, int val){
  // sets the piece of data to be requested from teensy
  byte ei = val & 0xFF;
  Wire.beginTransmission(address);  
  Wire.write(rq);                                              
  Wire.write(ei);                                                    
  Wire.endTransmission();
  last_rq = rq;
  last_ei = val;
}

void get_request(int address){
  Wire.requestFrom(address, 3);
  byte a, b, c;
  c = Wire.read();
  a = Wire.read();
  b = Wire.read();
  if (last_rq == 'K'){
    int bigNum;
    bigNum = a;
    bigNum = (bigNum << 8) | b;
    if (c == 0){
      bigNum = -1*bigNum;
    }
    // Serial.print("k_");
    // Serial.print(last_ei);
    // Serial.print(" = ");
    // Serial.println(bigNum);
    last_k = bigNum;
  }
  if (last_rq == 'A'){
    int DA = a;
    int RA = b;
    // Serial.print("DA_");
    // Serial.print(last_ei);
    // Serial.print(" = ");
    // Serial.print(DA);
    // Serial.print(", RA_");
    // Serial.print(last_ei);
    // Serial.print(" = ");
    // Serial.println(RA);
    last_DA = DA;
    last_RA = RA;
  }
  if (last_rq == 'B'){
    int DB = a;
    int RB = b;
    // Serial.print("DB_");
    // Serial.print(last_ei);
    // Serial.print(" = ");
    // Serial.print(DB);
    // Serial.print(", RB_");
    // Serial.print(last_ei);
    // Serial.print(" = ");
    // Serial.println(RB);
    last_DB = DB;
    last_RB = RB;
  }
}

// --------------------- Python Comms --------------------- 

void sendMessageWithVarToPython(const char* message, int var)
{
  // Serial.println(); // buffer line print
  Serial.print(buffer); // buffer characters
  Serial.print(divider);
  Serial.print("msgvar");
  Serial.print(divider);
  Serial.print(message);
  Serial.print(divider);
  Serial.print(var);
  Serial.print(divider);
  Serial.println(buffer); // buffer characters
}

void sendMessageToPython(const char* message) 
{
  // Serial.println(); // buffer line print
  Serial.print(buffer); // buffer characters
  Serial.print(divider);
  Serial.print("msg");
  Serial.print(divider);
  Serial.print(message);
  Serial.print(divider);
  Serial.println(buffer); // buffer characters
}


void send1DArrayToPython(const char* varname, int* arr, int len)
{
  Serial.print(buffer); // buffer characters
  Serial.print(divider);
  Serial.print("1Darr");
  Serial.print(divider);
  Serial.print(varname);
  Serial.print(divider);
  Serial.print("[");
  for (int i = 0; i < len; i++) {
    Serial.print(arr[i]);
    if (i < (len)-1){
      Serial.print(", ");
    }
    else{
      Serial.print("]");
    }
  }
  Serial.print(divider);
  Serial.println(buffer);
}

// ------- Helper Functions ---------------

// Helper funcion. Digital write function but faster than native digitalWrite
inline void digitalWriteDirect(int pin, boolean val) {
  if (val) g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;
  else g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin;
}

// Moves digipot by delta
void moveDigipot(int delta, int UDpin, int TICKpin) {
  if (delta >= 0) {
    digitalWriteDirect(UDpin, HIGH);  // move up
  } else {
    digitalWriteDirect(UDpin, LOW);  // move down
    delta = -delta;
  }

  for (int d = 0; d < delta; d++) {  // cycle clock desired number of times
    delayMicroseconds(digipot_delay_time);
    digitalWriteDirect(TICKpin, HIGH);
    delayMicroseconds(digipot_delay_time);
    digitalWriteDirect(TICKpin, LOW);
  }
}

void get_random_permutation(int N){
  for (int i=0; i < N; i++) {
    randomperm[i] = i;
  }

  for (int i=0; i < N; i++) {
    int n = random(0, N); 
    int temp = randomperm[n];
    randomperm[n] =  randomperm[i];
    randomperm[i] = temp;
  }
}

void populate_Ves(){
  int vi = 0;
  for (int i = 0; i < numE; i++){
    if (edgeNames[i].startsWith("BV")){
      Veidx[vi] = i;
      isAB[vi] = 1;
      whichV[vi] = edgeNames[i].substring(2).toInt();
      vi++;
    }
    if (edgeNames[i].startsWith("W")){
      Veidx[vi] = i;
      isAB[vi] = 0;
      whichV[vi] = edgeNames[i].substring(1,2).toInt();
      vi++;
    }
  }
  if (vi != numVe){
    sendMessageToPython("error wrong number of Ve");
  }
}