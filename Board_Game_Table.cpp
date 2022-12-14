#include <LiquidCrystal.h>

//Pins
#define remotePin 13

#define redOutPin 11
#define greenOutPin 12
#define blueOutPin 3

const byte sidePin[] = {13, 14, 15, 16, 17, 18}; //Next turn buttons

LiquidCrystal lcd0(8, 9, 4, 5, 6, 7);
LiquidCrystal lcd1(13, 13, 13, 13, 13, 13);
LiquidCrystal lcd2(13, 13, 13, 13, 13, 13);
LiquidCrystal lcd3(13, 13, 13, 13, 13, 13);
LiquidCrystal lcd4(13, 13, 13, 13, 13, 13);
LiquidCrystal lcd5(13, 13, 13, 13, 13, 13);
LiquidCrystal lcd[] = {lcd0, lcd1, lcd2, lcd3, lcd4, lcd5};

//IR Code should already be working

#define totalCodeLength 22
#define remoteCodeLength 17
#define buttonCodeLength 5
const bool remoteCode[] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1};


//Colors

const byte buttonColor[] =    {0, 0, 1, 2,   0, 3, 4, 5,   0, 6, 7, 8,   0, 9, 10, 11,   0, 0, 0, 0,   0, 0, 0, 0};
const byte buttonNumber[] =   {0, 1, 2, 3,   0, 4, 5, 6,   0, 7, 8, 9,   0, 0, 00, 00,   0, 0, 0, 0,   0, 0, 0, 0};

const byte r[] = {255, 125, 000, 153, 255, 165, 255, 000, 000, 000, 255, 127};
const byte g[] = {255, 125, 000, 076, 000, 127, 255, 255, 255, 000, 000, 000};
const byte b[] = {255, 125, 000, 000, 000, 000, 000, 000, 255, 255, 255, 255};
const String colorName[] = {"Whte", "Grey", "Blck", "Brwn", "Red", "Orng", "Yllw", "Grn", "Trqs", "Blue", "Pink", "Prpl"};

const String places[] = {"First", "Secnd", "Third", "Forth", "Fifth", "Sixth"};

//        Color     Index
#define   white     0
#define   grey      1
#define   black     2
#define   brown     3
#define   red       4
#define   orange    5
#define   yellow    6
#define   green     7
#define   turquise  8
#define   blue      9
#define   pink      10
#define   purple    11

//Code Variables
#define loopDelay 1  //msec
byte currentOrder = 0;  //For set order

//Game Settings
unsigned int fadeDelay = 2000;  //msec for light fade
unsigned int strobePeriod = 250;  //msec  for light shows
byte strobeDuration = 10;  //sec
#define messagePlayDuration 2000  //msec  //different if paused
#define messagePauseDuration 5000  //msec

//Global Variables
byte status = 0;

//Number  Mode
#define   paused            0
#define   play              1
#define   removePlayer      2
#define   setHost           3
#define   setPlayerColors   4
#define   setPlayerOrder          5
#define   setIndividualTimeLimit  6
#define   setGameTimeLimit        7
#define   selectWinner            8

byte currentPlayer = 0;
byte winner = 0;
bool reversed = false;
bool switchColors = true;  //false if single color
byte playColor = 0;
byte pauseColor = 0;
byte hostOffset = 0;  //Side of host
byte currentR = 0;
byte currentG = 0;
byte currentB = 0;
byte oldR = 0;
byte oldG = 0;
byte oldB = 0;

bool playerPlaying[] = {0, 1, 1, 1, 1, 1};  //0 if not playing
byte playerOrder[] = {0, 1, 2, 3, 4, 5};
byte playerColor[] = {white, red, white, white, white, white};
double playerTurnLength[] = {0, 0, 0, 0, 0, 0};  //for turn length prediction
unsigned int playerTurns[] = {0, 0, 0, 0, 0, 0};  //for turn length prediction
unsigned int individualTimeLimit = 3;  //sec
unsigned int gameTimeLimit = 0;  //min

unsigned long lastColorSwitchTime = 0;
unsigned long lastPlayerSwitchTime = 0;
unsigned long gameStartTime = 0;
unsigned long lastPauseTime = 0;
unsigned long lastStartTime = 0;
unsigned long lastStrobeTime = 0; 
unsigned long strobeStartTime = 0; 

byte currentButton = 0;
byte currentColor = 0;

byte message[] = {0, 0, 0, 0, 0, 0};
byte lastPlayerRemoved = 0;
long messageSentTime[] = {-100000, -100000, -100000, -100000, -100000, -100000};

//        Message             Index
#define   GameReset           0
#define   ColorSet            1
#define   OrderSet            2
#define   IndivTimeSet        3
#define   GameTimeSet         4
#define   PlayerWon           5
#define   YouWon              6
#define   PlayerRemoved       7
#define   YouHaveBeenRemoved  8
#define   CheatingReported    9
#define   OrderReversed       10
#define   PlayAdvanced        11
#define   CannotRemove        12
#define   PlayerAdded         14
#define   YouHaveBeenAdded    15


void setup() {
  pinMode(remotePin, INPUT_PULLUP);
  pinMode(redOutPin, OUTPUT);
  pinMode(greenOutPin, OUTPUT);
  pinMode(blueOutPin, OUTPUT);
  /*for (byte i = 0; i++; i < 6) {
    pinMode(sidePin[i], INPUT_PULLUP);
    delay(loopDelay);
  }*/
  lcd0.begin(16, 2);
  lcd1.begin(16, 2);
  lcd2.begin(16, 2);
  lcd3.begin(16, 2);
  lcd4.begin(16, 2);
  lcd5.begin(16, 2);
  for (byte i = 0; i++; i < 6) {
    lcd[i].clear();
    delay(loopDelay);
  }  
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(2, INPUT_PULLUP);
  randomSeed(analogRead(A1));
}

void loop() {
  //Update Remote
  /* Use this code for the IR Remote or use Serial Code Below to test inputs
    bool pulse[totalCodeLength];
    uint16_t highPulse = 0;
    uint16_t lowPulse = 0;
    bool stop = false;
    for(byte i=0; i<totalCodeLength && !stop; i++) {
      while (PIND & 4) {
        highPulse++;
        delayMicroseconds(20);
        if ((highPulse * 20 > 4700)) {
          stop=true;
          break;
        }
      }
      if(!stop) {
        if(highPulse < 1100) {
          pulse[i] = 0;
        }
        else {
          pulse[i] = 1;
        }
        while (!(bool)(PIND & 4)) {
          lowPulse++;
          delayMicroseconds(20);
          if ((lowPulse*20 >= 700)) {
            stop = true;
            break;
          }
        }
      }
    }
    if(!stop) {
      bool remoteCodeMatches = true;
      for(byte i=0; i < remoteCodeLength; i++) {
        if(remoteCode[i]==pulse[i+1]) {
          remoteCodeMatches = false;
          break;
        }
      }
      if(remoteCodeMatches) {
        byte code = 0;
        for(byte i=0; i < buttonCodeLength; i++) {
          code+=pulse[i+1+remoteCodeLength] << i;
        }
        if(code < 24) {
          remoteButtonPressed(code);
        }
      }
    }
  */
  if (Serial.available()) {
    byte code = Serial.parseInt();
    if(code < 24) {
      remoteButtonPressed(code);
    }
  }

  //Update Buttons and Turns
  if (status == play) {
    if(readButtonValue(sidePin[getCurrentSide()])) {
      advancePlay();
    }
    if(individualTimeLimit != 0 && individualTimeLimit - (millis() - lastPlayerSwitchTime) / 1000 <= 0) {
      advancePlay();
    }
    if(gameTimeLimit != 0 && gameTimeLimit - (millis() - gameStartTime) / 1000 <= 0) {
      status = paused;
    }
  }
  else if (status == setHost) { //Set Host
    for (byte i = 0; i < 6; i++) {
      if (readButtonValue(sidePin[i])) {
        hostOffset = i;
        status = paused;
        resetCurrentTurn();
        break;
      }
      delay(loopDelay);
    }
  }
  //Update Motors 
    //Not sure if you had servo motors?

  //Update Lights - "light show" for certain buttons
  //Select winner and report cheating (button 21 is cheating)
  if((status == selectWinner || currentButton == 21)
        &&  (millis() - strobeStartTime) / 1000 < strobeDuration) {
    if(millis() - lastStrobeTime < strobePeriod) {
      double multiplier = (double)random(50, 101) / 100;
      byte color = red;
      if(status == selectWinner) {
        if(switchColors) {
          color = playerColor[winner];
        }
        else {
          color = white;
        }
      }
      currentR = (r[color] + random(-20, 21)) * multiplier;
      currentG = (b[color] + random(-20, 21)) * multiplier;
      currentB = (g[color] + random(-20, 21)) * multiplier;
      lastStrobeTime = millis();
    }
  }
  else if(millis() - lastStartTime < 3000 && status == play) {  //Slow stobe after pressing play
    double multiplier = (millis() - lastStartTime) % 1000;
    multiplier = (double)(500 - abs(multiplier - 500)) / 500;
    byte color = playColor;
    if(switchColors) {
      color = playerColor[currentPlayer];
    }
    currentR = r[color] * multiplier;
    currentG = b[color] * multiplier;
    currentB = g[color] * multiplier;
  }
  else { //Single light colors
    byte targetColor;
    switch (status) {
      case paused:
        targetColor = pauseColor;
        break;
      case play:
        if (switchColors) {
          targetColor = playerColor[currentPlayer];
        }
        else {
          targetColor = playColor;
        }
        break;
      case removePlayer:
        targetColor = red;
        break;
      case setPlayerColors:
        if (currentPlayer == 0 || !playerPlaying[currentPlayer]) {
          targetColor = grey;
        }
        else {
          targetColor = playerColor[currentPlayer - 1];
        }
        break;
      case setPlayerOrder:
        if (currentOrder == 0) {
          targetColor = grey;
        }
        else {
          targetColor = playerColor[currentPlayer];
        }
        break;
      case selectWinner:
        targetColor = playerColor[winner];
        break;
      default:
        targetColor = grey;
        break;
    }
    if(currentButton == 21) {
      targetColor = red;
    }
    
    //Calculate RGB - Changes light color with 2 sec fade
    if (targetColor != currentColor) {
      lastColorSwitchTime = millis();
      oldR = currentR;
      oldG = currentG;
      oldB = currentB;
      currentColor = targetColor;
    }
    unsigned long elaspedTime = millis()  - lastColorSwitchTime;
    if (elaspedTime < fadeDelay) {
      currentR = r[currentColor] * elaspedTime                /  fadeDelay
                 + oldR          * (fadeDelay - elaspedTime)  /  fadeDelay;
      currentG = g[currentColor] * elaspedTime                /  fadeDelay
                 + oldG          * (fadeDelay - elaspedTime)  /  fadeDelay;
      currentB = b[currentColor] * elaspedTime                /  fadeDelay
                 + oldB          * (fadeDelay - elaspedTime)  /  fadeDelay;
    }
    else {
      currentR = r[currentColor];
      currentG = g[currentColor];
      currentB = b[currentColor];
    }
  }
  
  analogWrite(redOutPin, currentR);
  analogWrite(greenOutPin, currentG);
  analogWrite(blueOutPin, currentB);

  //Update LCDs
  for (byte i = 0; i < 6; i++) {
    byte player = (6 + i - hostOffset) % 6;

    //Host Message - statuses other than these 3 will have messages
    //Host Messages don't disapear and are used to set colors, players, etc
    bool isHostMessage = status != play 
                      && status != paused 
                      && status != setHost;

    //Message - these disapear
    bool isMessage = 0;
    int duration;
    if (status == play && playerPlaying[player]) {
      duration = messagePlayDuration;
    }
    else {
      duration = messagePauseDuration;
    }
    if (messageSentTime[player] > 0 && messageSentTime[player] + duration > millis()) {
      isMessage = 1;
    }
    
    //Print Line 1
      setLine(i, 0);
      
      //Host
      if (player == 0) {
        lcd[i].print("H ");
      }
      
      //Host Message or Player and Timer
      if(isHostMessage && isMessage) {
        printMessage(player, i);
      }
      else if(playerPlaying[player]) {
        //Player Name
        if (switchColors) {
          lcd[i].print(colorName[playerColor[player]]);
        }
        else {
          lcd[i].print(player + 1);
        }
        lcd[i].print(" ");
        
        //Indiv Timer  - if switch colors if off, there are no individual timers
        if (switchColors) {  //Determines what to display based on if ther is a time limit, if it is paused, and whose turn it is
          lcd[i].print("Ind-");
          unsigned long individualTimer = 0;
          if (individualTimeLimit == 0) {  //0s time limit counts up instead of down
            if (currentPlayer == player) {
              if(status != play) {
                individualTimer = (lastPauseTime - lastPlayerSwitchTime) / 1000;
              }
              else {
                individualTimer = (millis() - lastPlayerSwitchTime) / 1000;
              }
            }
            else {
              individualTimer = 0;
            }
          }
          else {
            if (currentPlayer == player) {
              if(status != play) {
                individualTimer = individualTimeLimit - (lastPauseTime - lastPlayerSwitchTime) / 1000;
              }
              else {
                individualTimer = individualTimeLimit - (millis() - lastPlayerSwitchTime) / 1000;
              }
            }
            else {
              individualTimer = individualTimeLimit;
            }
          }
          
          if(individualTimer >= 60) {
            lcd[i].print(individualTimer / 60);
            lcd[i].print("m");
          }
          lcd[i].print(individualTimer % 60);
          lcd[i].print("s");
        }
      }
      lcd[i].print("                ");
    
    //Print Line 2
      setLine(i, 1);
      
      //Host Message or Message or Timers
      if(isHostMessage) {
        switch (status) {
          case removePlayer:
            lcd[i].print("Add/Remove Plyer");
            break;
          case setPlayerColors:
            lcd[i].print("Set Plyr ");
            lcd[i].print(currentPlayer + 1);
            lcd[i].print("s Colr");
            break;
          case setPlayerOrder:
            lcd[i].print("Selct ");
            lcd[i].print(places[currentOrder]);
            lcd[i].print(" Plyr");
            break;
          case setIndividualTimeLimit:
            lcd[i].print("Set Indiv-");
            lcd[i].print(individualTimeLimit);
            lcd[i].print("s");
            break;
          case setGameTimeLimit:
            lcd[i].print("Set Game-");
            lcd[i].print(gameTimeLimit);
            lcd[i].print("m");
            break;
          case selectWinner:
            lcd[i].print("Select Winner");
            break;
        }
      }
      else if(isMessage) {
        printMessage(player, i);
      }
      else if(playerPlaying[player]) {
        //Calculate Turns
        byte totalTurns = 0;
        double averageTurnLength = 0;
        for(int j=0; j<6; j++) {
          totalTurns += playerTurns[j];
          averageTurnLength += playerTurnLength[j] * playerTurns[j];
        }
        if(totalTurns != 0) {
          averageTurnLength /= totalTurns;
        }
        
        //Wait Timer - predicted wait time
        if(switchColors && totalTurns >= getPlayersPlaying()) {
          unsigned int waitTimer = 0;
          if (currentPlayer == player) {
            for (byte j = 0; j < 6; j++) {
              if (playerPlaying[i] && j != player) {
                if(playerTurnLength[j] > 0) {
                  waitTimer += playerTurnLength[j];
                }
                else {
                  waitTimer += averageTurnLength;
                }
              }
              delay(loopDelay);
            }
          }
          else {
            int currentPlayerWaitTime = 0;
            if(status != play) {
              if(playerTurnLength[currentPlayer] > 0) {
                currentPlayerWaitTime = playerTurnLength[currentPlayer] - (lastPauseTime - lastPlayerSwitchTime) / 1000;
              }
              else {
                currentPlayerWaitTime = averageTurnLength - (lastPauseTime - lastPlayerSwitchTime) / 1000;
              }
            }
            else {
              if(playerTurnLength[currentPlayer] > 0) {
                currentPlayerWaitTime = playerTurnLength[currentPlayer] - (millis() - lastPlayerSwitchTime) / 1000;
              }
              else {
                currentPlayerWaitTime = averageTurnLength - (millis() - lastPlayerSwitchTime) / 1000;
              }
            }
            if(currentPlayerWaitTime > 0) {
              waitTimer += currentPlayerWaitTime;
            }
            byte j = (getPlayerPlayingOrder(currentPlayer) + 1) % getPlayersPlaying();
            while(j != getPlayerPlayingOrder(player)) {
              for(int k=0; k<6; k++) {
                if(getPlayerPlayingOrder(k) == j) {
                  if(playerTurnLength[k] > 0) {
                    waitTimer += playerTurnLength[k];
                  }
                  else {
                    waitTimer += averageTurnLength;
                  }
                  break;
                }
                delay(loopDelay);
              }
              if(reversed) {
                j -= 1;
              }
              else {
                j += 1;
              }
              j %= getPlayersPlaying();
              delay(loopDelay);
            }
          }
          lcd[i].print("Wt-");
          
          if(waitTimer >= 60) {
            lcd[i].print(waitTimer / 60);
            lcd[i].print("m");
          }
          lcd[i].print(waitTimer % 60);
          lcd[i].print("s ");
        }
  
        //Game Timer - game time - if 0, counts up
        int gameTimer;
        if (gameTimeLimit == 0) {
          if(status != play) {
            gameTimer = (lastPauseTime - gameStartTime) / 60000;
          }
          else {
            gameTimer = (millis() - gameStartTime) / 60000;
          }
        }
        else {
          if(status != play) {
            gameTimer = gameTimeLimit - (lastPauseTime - gameStartTime) / 60000;
          }
          else {
            gameTimer = gameTimeLimit - (millis() - gameStartTime) / 60000;
          }
        }
        lcd[i].print("Gm-");
        if(gameTimer >= 60) {
          lcd[i].print(gameTimer / 60);
          lcd[i].print("h");
        }
        lcd[i].print(gameTimer % 60);
        lcd[i].print("m");
      }
      lcd[i].print("                ");
    
    delay(loopDelay);
  }
  
  delay(loopDelay);
}

//updates things if button is pressed
void remoteButtonPressed(byte button) {
  byte oldStatus = status;
  if (status == play) {
    lastPauseTime = millis();
  }
  else {
    lastStartTime = millis();
  }
  if(status != selectWinner || currentButton != 21) {
    strobeStartTime = millis();
  }
  currentButton == button;
  switch (currentButton) {
    case 0: //Reset Game
      gameStartTime = millis();
      lastPauseTime = millis();
      lastPlayerSwitchTime = millis();
      for (byte i = 0; i < 6; i++) {
        playerTurnLength[i] = 0;
        playerTurns[i] = 0;
        delay(loopDelay);
      }
      //average times and counters
      status = paused;
      resetCurrentTurn();
      sendGlobalMessage(GameReset);
      break;
    case 4: //Set Host
      if (status == setHost) {
        status = paused;
      }
      else {
        status = setHost;
      }
      break;
    case 8: //Set Colors
      if (status == play && !switchColors) {
        switchColors = true;
      }
      else if (status == setPlayerColors) {
        status = paused;
      }
      else {
        status = setPlayerColors;
        currentPlayer = 0;
      }
      break;
    case 12:  //Set Order
      if (status == setPlayerOrder) {
        status = paused;
      }
      else {
        status = setPlayerOrder;
        reversed = false;
        currentOrder = 0;
      }
      break;
    case 16:  //Set Individual Time Limit
      if (status == setIndividualTimeLimit) {
        status = paused;
      }
      else {
        status = setIndividualTimeLimit;
        individualTimeLimit = 0;
      }
      break;
    case 17:  //Set Game Time Limit
      if (status == setGameTimeLimit) {
        status = paused;
      }
      else {
        status = setGameTimeLimit;
        gameTimeLimit = 0;
      }
      break;
    case 18:  //Select Winner
      if (status == selectWinner) {
        status = paused;
      }
      else {
        status = selectWinner;
      }
      break;
    case 19:  //Remove Player
      if (status == setPlayerColors) {
        if(Remove(currentPlayer)) {
          if (currentPlayer >= 5) {
            status = paused;
            resetCurrentTurn();
          }
          else {
            currentPlayer++;
          }
        }
      }
      else if (status == removePlayer) {
        status = paused;
      }
      else {
        status = removePlayer;
      }
      break;
    case 20:  //Play/Pause
      if (status == play) {
        status = paused;
      }
      else {
        status = play;
        gameStartTime += millis() - lastPauseTime;
        lastPlayerSwitchTime += millis() - lastPauseTime;
      }
      break;
    case 21:  //Report Cheating
      status = paused;
      sendGlobalMessage(CheatingReported);
      break;
    case 22:  //Reverse Order
      reversed = !reversed;
      sendGlobalMessage(OrderReversed);
      break;
    case 23:  //Advance Play
      advancePlay();
      sendMessage(0, PlayAdvanced);
      break;
    default:  //Numbers/Colors
      switch (status) {
        case paused: //Paused
          pauseColor = buttonColor[currentButton];
          break;
        case play: //Play
          pauseColor = buttonColor[currentButton];
          switchColors = false;
          break;
        case removePlayer: //Remove Player
          if (currentButton <= 7) {
            if(playerPlaying[buttonNumber[currentButton] - 1]) {
              Remove(buttonNumber[currentButton] - 1);
            }
            else {
              playerPlaying[buttonNumber[currentButton] - 1] = 1;
              sendGlobalMessage(PlayerAdded);
              sendMessage(buttonNumber[currentButton] - 1, YouHaveBeenAdded);
            }      
          }
          status = paused;
          break;
        case setHost: //Set Host
          status = paused;
          break;
        case setPlayerColors: //Set Player Colors
          playerColor[currentPlayer] = buttonColor[currentButton];
          playerPlaying[currentPlayer] = 1;
          sendMessage(currentPlayer, ColorSet);

          if (currentPlayer >= 5) {
            status = paused;
            resetCurrentTurn();
          }
          else {
            currentPlayer++;
          }
          break;
        case setPlayerOrder: //Set Player Order
          if (currentButton <= 7) {
            currentPlayer = buttonNumber[currentButton] - 1;
            playerPlaying[currentPlayer] = 1;
            setOrder(currentPlayer, currentOrder);
            sendMessage(currentPlayer, OrderSet);
            if (currentOrder >= getPlayersPlaying() - 2) {
              for(int i=0; i<6; i++) {
                if(getPlayerPlayingOrder(i) >= getPlayersPlaying() - 1) {
                  setOrder(i, currentOrder + 1);
                  sendMessage(i, OrderSet);
                  break;
                }
              }
              status = paused;
              resetCurrentTurn();
            }
            else {
              currentOrder++;
            }
          }
          else {
            status = paused;
            resetCurrentTurn();
          }
          break;
        case setIndividualTimeLimit: //Set Individual Time Limit
          if (currentButton == 15) {
            status = paused;
          }
          else if (currentButton == 13) {
            individualTimeLimit *= 100;
          }
          else {
            individualTimeLimit *= 10 ;
            individualTimeLimit += buttonNumber[currentButton];
            p(0);
          }
          break;
        case setGameTimeLimit: //Set Game Time Limit
          if (currentButton == 15) {
            status = paused;
          }
          else if (currentButton == 13) {
            gameTimeLimit *= 100;
          }
          else {
            gameTimeLimit *= 10;
            gameTimeLimit += buttonNumber[currentButton];
          }
          break;
        case selectWinner:
          if (currentButton <= 7) {
            winner = buttonNumber[currentButton] - 1;
            sendGlobalMessage(PlayerWon);
            sendMessage(winner, YouWon);
            resetCurrentTurn();
          }
          break;
      }
      break;
  }
  if (oldStatus == setIndividualTimeLimit && status != setIndividualTimeLimit) {
    sendGlobalMessage(IndivTimeSet);
  }
  else if (oldStatus == setGameTimeLimit && status != setGameTimeLimit) {
    sendGlobalMessage(GameTimeSet);
  }
}

//next player - reversed if reversed
void advancePlay() {
  playerTurnLength[currentPlayer] = (playerTurnLength[currentPlayer] * playerTurns[currentPlayer] 
                                        + (double)(millis() - lastPlayerSwitchTime)/1000) 
                                      / (playerTurns[currentPlayer] + 1);
  playerTurns[currentPlayer] += 1;
  lastPlayerSwitchTime = millis();
  
  int8_t nextOrder;
  if (!reversed) {
    nextOrder = getPlayerPlayingOrder(currentPlayer) + 1;
  }
  else {
    nextOrder = getPlayerPlayingOrder(currentPlayer) - 1;
  }
  
  nextOrder += getPlayersPlaying();
  nextOrder %= getPlayersPlaying();
  
  for (byte i = 0; i < 6; i++) {
    if (getPlayerPlayingOrder(i) == nextOrder) {  
      currentPlayer = i;
      break;
    }
    delay(loopDelay);
  }
}

//excludes players not playing
byte getPlayerPlayingOrder(byte player) {
  if(!playerPlaying[player]) {
    return -1;
  }
  byte order = 0;
  for(int i=0; i<6; i++) {
    order += playerPlaying[i] && playerOrder[i] < playerOrder[player];
  }
  return order;
}

byte getPlayersPlaying() {
  byte playersPlaying = 0;
  for (byte i = 0; i < 6; i++) {
    playersPlaying += playerPlaying[i];
    delay(loopDelay);
  }
  return playersPlaying;
}

//goes back to whoever starts the game
void resetCurrentTurn() {
  if (!reversed) {
    byte min = 6;
    byte index = 0;
    for (byte i = 0; i < 6; i++) {
      if (playerPlaying[i] && playerOrder[i] < min) {
        min = playerOrder[i];
        index = i;
      }
      delay(loopDelay);
    }
    currentPlayer = index;
  }
  else {
    byte max = -1;
    byte index = 0;
    for (byte i = 0; i < 6; i++) {
      if (playerPlaying[i] && playerOrder[i] > max) {
        max = playerOrder[i];
        index = i;
      }
      delay(loopDelay);
    }
    currentPlayer = index;
  }
}

//adjusts for host offset
byte getCurrentSide() {
  return (currentPlayer + hostOffset) % 6;
}

void sendMessage(byte player, byte mess) {
  message[player] = mess;
  messageSentTime[player] = millis();
}

void sendGlobalMessage(byte message) {
  for (byte i = 0; i < 6; i++) {
    sendMessage(i, message);
    delay(loopDelay);
  }
}

bool readButtonValue(byte side) {
  int values[] = {1023, 823, 624, 406, 206, 0, -500};
  side -= 13;
  if (side == 5) {
    return !digitalRead(2);
  }
  else {
    int data = analogRead(A0);
    if (abs(data - values[side + 1]) < abs(values[side + 2] - data) && abs(data - values[side + 1]) < abs(values[side] - data)) {
      return 1;
    }
    else {
      return 0;
    }
  }
}

//serial prints value
void p(byte num) {
  Serial.print(num);
  delay(5);
}

void p(String str) {
  Serial.print(str);
  delay(5);
}

void setLine(byte lcd, bool line) {
  switch(lcd) {
    case 0:
      lcd0.setCursor(0, line);
      break;
    case 1:
      lcd1.setCursor(0, line);
      break;
    case 2:
      lcd2.setCursor(0, line);
      break;
    case 3:
      lcd3.setCursor(0, line);
      break;
    case 4:
      lcd4.setCursor(0, line);
      break;
    case 5:
      lcd5.setCursor(0, line);
      break;
  }
}

//rearranges order to prevent duplicates
void setOrder(byte player, byte order) {
  byte oldOrder = playerOrder[player];
  if(oldOrder > order) {
    for (byte i = oldOrder; i > order; i--) {
      for (byte j = 0; j < 6; j++) {
        if (playerOrder[j] == i - 1) {
          playerOrder[j] = i;
          break;
        }
      delay(loopDelay);
      }
      delay(loopDelay);
    }
  }
  else if(oldOrder < order) {
    for (byte i = oldOrder; i < order; i++) {
      for (byte j = 0; j < 6; j++) {
        if (playerOrder[j] == i + 1) {
          playerOrder[j] = i;
          break;
        }
        delay(loopDelay);
      }
      delay(loopDelay);
    }
  }  
  playerOrder[player] = order;          
}

//removes player
bool Remove(byte player) {
  if(getPlayersPlaying() > 1) {
    playerPlaying[player] = 0;
    if(status == removePlayer) {
      sendGlobalMessage(PlayerRemoved);
    }
    sendMessage(player, YouHaveBeenRemoved);
    lastPlayerRemoved = player;
    return 1;
  }
  else {
    sendMessage(0, CannotRemove);
    return 0;
  }
}

void printMessage(byte player, byte i) {
  switch(message[player]) {
    case GameReset:
      lcd[i].print("Game Reset");
      break;
    case ColorSet:
      lcd[i].print("Color Set-");
      lcd[i].print(colorName[playerColor[player]]);
      break;
    case OrderSet:
      lcd[i].print("Order Set-");
      lcd[i].print(getPlayerPlayingOrder(player) + 1);
      break;
    case IndivTimeSet:
      lcd[i].print("Indiv Set-");
      lcd[i].print(individualTimeLimit);
      lcd[i].print("s");
      break;
    case GameTimeSet:
      lcd[i].print("Game Set-");
      lcd[i].print(gameTimeLimit);
      lcd[i].print("m");
      break;
    case PlayerWon:
      if(switchColors) {
        lcd[i].print(colorName[playerColor[winner]]);
        lcd[i].print(" Won!");
      }
      else {
        lcd[i].print("Player ");
        lcd[i].print(winner + 1);
        lcd[i].print(" Won!");
      }
      break;
    case YouWon:
      lcd[i].print("You Won!");
      break;
    case PlayerRemoved:
      if(switchColors) {
        lcd[i].print(colorName[playerColor[lastPlayerRemoved]]);
        lcd[i].print(" Removed");
      }
      else {
        lcd[i].print("Player ");
        lcd[i].print(winner + 1);
        lcd[i].print(" Removed");
      }          
      break;
    case YouHaveBeenRemoved:
      lcd[i].print("You're Removed");
      break;
    case CheatingReported:
      lcd[i].print("Cheating Rprted");
      break;
    case OrderReversed:
      lcd[i].print("Order Reversed");
      break;
    case PlayAdvanced:
      lcd[i].print("Play Advanced");
      break;
    case CannotRemove:
      lcd[i].print("Cannot Remove");
      break;
    case PlayerAdded:
      if(switchColors) {
        lcd[i].print(colorName[playerColor[lastPlayerRemoved]]);
        lcd[i].print(" Added");
      }
      else {
        lcd[i].print("Player ");
        lcd[i].print(winner + 1);
        lcd[i].print(" Added");
      }          
      break;
    case YouHaveBeenAdded:
      lcd[i].print("You're Added");
      break;
  }
}
