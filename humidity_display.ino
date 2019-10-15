/*
   Convert RF signal into bits (humidity/temperature sensor version)
     Written by : Ray Wang (Rayshobby LLC)
     http://rayshobby.net/?p=8998
*/

// ring buffer size has to be large enough to fit
// data between two successive sync signals
#define RING_BUFFER_SIZE  256
#define SYNC_LENGTH       2200
#define SYNC_HIGH         600
#define SYNC_LOW          600
#define BIT1_HIGH         400
#define BIT1_LOW          220
#define BIT0_HIGH         220
#define BIT0_LOW          400
#define DATAPIN           3  // D3 is interrupt 1

unsigned long timings[RING_BUFFER_SIZE];
unsigned int syncIndex1 = 0;  // index of the first sync signal
unsigned int syncIndex2 = 0;  // index of the second sync signal
bool received = false;

/*
    Detect if a sync signal is present
*/
bool isSync(unsigned int idx) {
  // check if we've received 4 squarewaves of matching timing
  int i;
  for (i = 0; i < 8; i += 2) {
    unsigned long t1 = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
    unsigned long t0 = timings[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];
    if (t0 < (SYNC_HIGH - 100) || t0 > (SYNC_HIGH + 100) ||
        t1 < (SYNC_LOW - 100)  || t1 > (SYNC_LOW + 100)) {
      return false;
    }
  }
  // check if there is a long sync period prior to the 4 squarewaves
  unsigned long t = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
  if (t < (SYNC_LENGTH - 400) || t > (SYNC_LENGTH + 400) ||
      digitalRead(DATAPIN) != HIGH) {
    return false;
  }
  return true;
}

/* Interrupt 1 handler */
void handler() {
  Serial.println(F("Handler called."));
  Serial.print(F("recieved="));
  Serial.println(received);

  static unsigned long duration = 0, lastTime = 0;
  static unsigned int ringIndex = 0, syncCount = 0;

  // ignore if we haven't processed the previous received signal
  if (received == true)
    return;

  // calculating timing since last change
  long time = micros();
  duration = time - lastTime;
  lastTime = time;

  // store data in ring buffer
  ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  timings[ringIndex] = duration;

  // detect sync signal
  if (isSync(ringIndex))
  {
    syncCount ++;
    // first time sync is seen, record buffer index
    if (syncCount == 1)
      syncIndex1 = (ringIndex + 1) % RING_BUFFER_SIZE;

    else if (syncCount == 2) {
      // second time sync is seen, start bit conversion
      syncCount = 0;
      syncIndex2 = (ringIndex + 1) % RING_BUFFER_SIZE;
      unsigned int changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2 + RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
      // changeCount must be 122 -- 60 bits x 2 + 2 for sync
      if (changeCount != 122) {
        received = false;
        syncIndex1 = 0;
        syncIndex2 = 0;
      } else
        received = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Humidity display Started.");
  pinMode(DATAPIN, INPUT);
  attachInterrupt(1, handler, CHANGE);
}

int t2b(unsigned int t0, unsigned int t1) {
  if (t0 > (BIT1_HIGH - 100) && t0 < (BIT1_HIGH + 100) && t1 > (BIT1_LOW - 100) && t1 < (BIT1_LOW + 100))
    return 1;
  else if (t0 > (BIT0_HIGH - 100) && t0 < (BIT0_HIGH + 100) && t1 > (BIT0_LOW - 100) && t1 < (BIT0_LOW + 100))
    return 0;
  return -1;  // undefined
}

void loop() 
{
  if (received == true) 
  {
    // disable interrupt to avoid new data corrupting the buffer
    detachInterrupt(1);

    // extract humidity value
    unsigned int startIndex, stopIndex;
    unsigned long humidity = 0;
    bool fail = false;
    startIndex = (syncIndex1 + (3 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    stopIndex =  (syncIndex1 + (3 * 8 + 8) * 2) % RING_BUFFER_SIZE;

    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE) {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      humidity = (humidity << 1) + bit;
      if (bit < 0)  fail = true;
    }
    if (fail) {
      Serial.println(F("Decoding error."));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.print("% / ");
    }

    // extra temperature value
    unsigned long temp = 0;
    fail = false;
    // most significant 4 bits
    startIndex = (syncIndex1 + (4 * 8 + 4) * 2) % RING_BUFFER_SIZE;
    stopIndex  = (syncIndex1 + (4 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE) {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      temp = (temp << 1) + bit;
      if (bit < 0)  fail = true;
    }
    // least significant 7 bits
    startIndex = (syncIndex1 + (5 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    stopIndex  = (syncIndex1 + (5 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE) {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      temp = (temp << 1) + bit;
      if (bit < 0)  fail = true;
    }

    if (fail) {
      Serial.println(F("Decoding error."));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print((int)((temp - 1024) / 10 + 1.9 + 0.5)); // round to the nearest integer
      Serial.write(176);    // degree symbol
      Serial.print("C/");
      Serial.print((int)(((temp - 1024) / 10 + 1.9 + 0.5) * 9 / 5 + 32)); // convert to F
      Serial.write(176);    // degree symbol
      Serial.println("F");
    }
    // delay for 1 second to avoid repetitions
    delay(1000);
    received = false;
    syncIndex1 = 0;
    syncIndex2 = 0;

    // re-enable interrupt
    attachInterrupt(1, handler, CHANGE);
  }

}

