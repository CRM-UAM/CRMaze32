#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "EEPROM.h"
#include "floodfill.h"

short distances[16][16]={0};
byte walls[16][16]={{14,8,8,8,8,8,8,8,8,8,8,8,8,8,8,9},{12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},{6,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3}};
byte visited[16][16]={0};

//CAMM 2012 Maze
//#define SIMULATED_MAZE
#ifdef SIMULATED_MAZE
byte wallsSim[16][16]={{14,8,9,13,13,13,12,10,10,10,9,12,10,8,10,9},{12,3,4,0,0,0,1,14,8,10,3,5,12,2,9,5},{6,9,5,7,7,7,6,9,6,8,10,3,5,12,3,5},{12,3,6,10,10,10,9,6,9,7,12,9,5,6,9,5},{6,9,12,10,10,10,2,11,6,10,3,6,3,12,3,5},{12,2,1,14,8,10,10,10,10,10,10,10,10,2,9,5},{5,13,4,9,5,12,8,8,11,14,8,10,8,11,5,5},{5,4,3,5,5,5,5,4,9,12,3,12,2,9,5,5},{5,5,12,3,5,5,5,6,3,6,9,6,8,3,5,5},{5,5,6,9,5,5,5,14,8,9,5,12,2,9,5,5},{5,5,12,2,1,5,4,9,5,4,3,6,8,3,5,5},{4,0,2,8,3,4,3,6,1,5,12,9,6,9,5,5},{5,5,12,3,14,2,10,10,2,1,5,6,9,6,1,5},{6,2,3,12,9,12,10,10,10,3,6,9,6,9,5,5},{12,10,10,3,6,2,10,10,10,10,10,2,11,6,1,5},{6,10,10,10,10,10,10,10,10,10,10,10,10,10,2,3}};
#endif

extern hw_timer_t * timer;
int EEPROM_save_maze(){
      timerAlarmDisable(timer);
      delay(10);
      if (!EEPROM.begin(256)){
            timerAlarmEnable(timer);
            return -1;
      }
      int addr = 0;
      for(int i =0;i<16;i++){
            for(int j=0;j<16;j++){
                  if(visited[i][j])
                        EEPROM.write(addr, byte(walls[i][j] | 0x10));
                  else
                        EEPROM.write(addr, byte(walls[i][j] & 0x0F));
                  addr++;
            }
      }
      EEPROM.commit();
      timerAlarmEnable(timer);
      return addr;
}


int EEPROM_load_maze(){
      timerAlarmDisable(timer);
      delay(10);
      if (!EEPROM.begin(256)){
            timerAlarmEnable(timer);
            return -1;
      }
      int addr = 0;
      for(int i =0;i<16;i++){
            for(int j=0;j<16;j++){
                  byte val = byte(EEPROM.read(addr));
                  if(val & 0x10) //visited
                        visited[i][j]=1;
                  else
                        visited[i][j]=0;

                  walls[i][j]= val & 0x0F;
                  addr++;
            }
      }
      timerAlarmEnable(timer);
      return addr;
}


/*short mazeQueue[512]={0};*/
short mazeStack[512]={0};
int stackPointer=0;
int enqueuePointer=1;
int dequeuePointer=0;

void enqueueCell(short cell){
      if(dequeuePointer==enqueuePointer)
            return;
      mazeStack[enqueuePointer]=cell;
      enqueuePointer++;
      enqueuePointer= enqueuePointer%512;
}
short dequeueCell(){
      if(dequeuePointer==enqueuePointer-1)
            return -1;
      dequeuePointer++;
      if(dequeuePointer>=512){
            dequeuePointer=0;
            return mazeStack[511];
      }
      return mazeStack[dequeuePointer];
}

void pushCell(short c){
      if(stackPointer<512){
//            if(c <0 )Serial.println(" PUSHED NEGATIVE VALUE");
          mazeStack[stackPointer]=c;
          stackPointer++;
      }
}
short popCell(){
      if(stackPointer<=0){
            return -1;
      }
      stackPointer--;
      return mazeStack[stackPointer];

}



void printMaze(short xRobot, short yRobot, short dirRobot){
      int i=0;
      int j=0;
      for(j=15;j>=0;j--){
            for(i=0;i<16;i++){
                  Serial.print("+");
                  if(hasTopWall(walls[i][j])!=0)
                        Serial.print("---");
                  else
                        Serial.print("   ");
            }
            Serial.println("+");

            for(i=0;i<16;i++){
                  if(hasLeftWall(walls[i][j])!=0)
                        Serial.print("|");
                  else
                        Serial.print(" ");
                  if(i==xRobot && j==yRobot){
                        switch(dirRobot){
                              case UP_DIR:
                                    Serial.print("^");break;
                              case DOWN_DIR:
                                    Serial.print("v");break;
                              case RIGHT_DIR:
                                    Serial.print(">");break;
                              case LEFT_DIR:
                                    Serial.print("<");break;
                        }
                  }else{
                        if(visited[i][j])
                              Serial.print("*");
                        else
                              Serial.print(" ");
                  }

                  Serial.print(distances[i][j]);
                  if(distances[i][j]<10)
                        Serial.print(" ");
            }
            if(hasRightWall(walls[15][j])!=0)
                        Serial.println("|");
                  else
                        Serial.println(" ");

      }
      for(i=0;i<16;i++){
            Serial.print("+");
            if(hasBottomWall(walls[i][0])!=0)
                  Serial.print("---");
            else
                  Serial.print("   ");
      }
      Serial.println("+");
}

void updateCurrentWalls(short xRobot, short yRobot, short wallsRobot){
      if(hasTopWall(wallsRobot) && yRobot<15){
            walls[xRobot][yRobot]= insertTopWall(walls[xRobot][yRobot]);
            walls[xRobot][yRobot+1]= insertBottomWall(walls[xRobot][yRobot+1]);
      }
      if(hasRightWall(wallsRobot) && xRobot<15){
            walls[xRobot][yRobot]= insertRightWall(walls[xRobot][yRobot]);
            walls[xRobot+1][yRobot]= insertLeftWall(walls[xRobot+1][yRobot]);
      }
      if(hasBottomWall(wallsRobot) && yRobot>0){
            walls[xRobot][yRobot]= insertBottomWall(walls[xRobot][yRobot]);
            walls[xRobot][yRobot-1]= insertTopWall(walls[xRobot][yRobot-1]);
      }
      if(hasLeftWall(wallsRobot) && xRobot>0){
            walls[xRobot][yRobot]= insertLeftWall(walls[xRobot][yRobot]);
            walls[xRobot-1][yRobot]= insertRightWall(walls[xRobot-1][yRobot]);
      }
}

short  FFUpdateDistances(short xRobot, short yRobot){
      if(visited[xRobot][yRobot]) //already runned floodfill
            return -1;
      int cont=0;
      //Make sure stack is empty
      while(popCell()!=-1){
            Serial.println("Error");
      }
//      Serial.print("Debug FF upd: ");Serial.println(wallsRobot);
      //enqueue current cell and nadjacents to the new walls and update the maze matrix
      short wallsRobot=walls[xRobot][yRobot];
      if(hasTopWall(wallsRobot) && yRobot<15){
            pushCell(CellToStack(xRobot,(yRobot+1)));
            cont++;
//            Serial.print("Top Wall Updated: ");Serial.println(walls[xRobot][yRobot]);
      }
      if(hasRightWall(wallsRobot) && xRobot<15){
            pushCell(CellToStack((xRobot+1),yRobot));
            cont++;
//            Serial.print("Right Wall Updated: "); Serial.println(walls[xRobot][yRobot]);
      }
      if(hasBottomWall(wallsRobot) && yRobot>0){
            pushCell(CellToStack(xRobot,(yRobot-1)));
            cont++;
//            Serial.print("Bottom Wall Updated: ");Serial.println(walls[xRobot][yRobot]);
      }
      if(hasLeftWall(wallsRobot) && xRobot>0){
            pushCell(CellToStack((xRobot-1),yRobot));
            cont++;
//            Serial.print("Left Wall Updated: ");Serial.println(walls[xRobot][yRobot]);
      }
      pushCell(CellToStack(xRobot,yRobot));

      //pop cell to the stack until empty
      short currCell=popCell();
      while(currCell>=0){
            short x=StackToCellX(currCell);
            short y=StackToCellY(currCell);
            short currWalls=walls[x][y];
//           Serial.print("Curr Cell: ");Serial.print(x);Serial.print(" ");Serial.print(y);Serial.print(" walls: ");Serial.println(currWalls);
            //update distance to the minimum accesible neighbors +1
//            Serial.print("Distance right: ");Serial.print(distances[x+1][y]); Serial.print(" walls right: ");Serial.println(hasRightWall(currWalls));
            int minDistance = 10000;
            if(y<15 && !hasTopWall(currWalls) && distances[x][y+1]<minDistance)
                  minDistance=distances[x][y+1];
            if(y>0 && !hasBottomWall(currWalls) && distances[x][y-1]<minDistance)
                  minDistance=distances[x][y-1];
            if(x<15 && !hasRightWall(currWalls) && distances[x+1][y]<minDistance)
                  minDistance=distances[x+1][y];
            if(x>0 && !hasLeftWall(currWalls) && distances[x-1][y]<minDistance)
                  minDistance=distances[x-1][y];
//            Serial.print("neighbors min dist: ");Serial.print(minDistance);Serial.print(" Cell Disctance: ");Serial.println(distances[x][y]);Serial.print(" ");
            if(distances[x][y] > 0 && distances[x][y] != minDistance+1){ //update distance of current cell and stack all accesible neighbors
                  distances[x][y] = minDistance+1;
                  if(y<15 && !hasTopWall(currWalls)){
                        pushCell(CellToStack((x),(y+1)));
                        cont++;
//                        Serial.print("Pushed Top ");
                  }
                  if(y>0 && !hasBottomWall(currWalls)){
                        pushCell(CellToStack((x),(y-1)));
                        cont++;
//                        Serial.print("Pushed Bottom ");
                  }
                  if(x<15 && !hasRightWall(currWalls)){
                        pushCell(CellToStack((x+1),(y)));
                        cont++;
//                        Serial.print("Pushed Right ");
                  }
                  if(x>0 && !hasLeftWall(currWalls)){
                        pushCell(CellToStack((x-1),(y)));
                        cont++;
//                        Serial.print("Pushed Left ");
                  }
//                  Serial.println(" ");
            }else{
//                  Serial.println("Not distance update necesary");
            }

            //pop new cell until empty
//            Serial.print("Stack size: ");Serial.println(stackPointer);
            currCell=popCell();
      }

      visited[xRobot][yRobot]=1;
      return cont;
}

void initializeDistancesCenterGoal(){
      for(int i=0;i<16;i++)
            for(int j=0;j<16;j++){
                  int dX=abs(i-8) < abs(i-7) ? abs(i-8):abs(i-7);
                  int dY=abs(j-8) < abs(j-7) ? abs(j-8):abs(j-7);
                  distances[i][j]=dX+dY;
                  visited[i][j]=0;
            }
}
//private
void updateDistancesWithQueue(){
      short currCell=dequeueCell();
      while(currCell>=0){

            short x=StackToCellX(currCell);
            short y=StackToCellY(currCell);
            if(visited[x][y]==0){
                  short currWalls=walls[x][y];
                  int minDistance = 9999;
//                  Serial.print("Push cell: ");
                  if(y<15 && !hasTopWall(currWalls) ){
                        if(distances[x][y+1]<minDistance)
                              minDistance=distances[x][y+1];
                        enqueueCell(CellToStack(x,y+1));
//                        Serial.print(x);Serial.print(" ");Serial.print(y+1);Serial.print("  , ");
                  }
                  if(y>0 && !hasBottomWall(currWalls)){
                        if(distances[x][y-1]<minDistance)
                              minDistance=distances[x][y-1];
                        enqueueCell(CellToStack(x,y-1));
//                        Serial.print(x);Serial.print(" ");Serial.print(y-1);Serial.print("  , ");
                  }
                  if(x<15 && !hasRightWall(currWalls)){
                        if(distances[x+1][y]<minDistance)
                              minDistance=distances[x+1][y];
                        enqueueCell(CellToStack(x+1,y));
//                        Serial.print(x+1);Serial.print(" ");Serial.print(y);Serial.print("  , ");
                  }
                  if(x>0 && !hasLeftWall(currWalls)){
                        if(distances[x-1][y]<minDistance)
                              minDistance=distances[x-1][y];
                        enqueueCell(CellToStack(x-1,y));
 //                       Serial.print(x-1);Serial.print(" ");Serial.print(y);Serial.print("  , ");
                  }
//                  Serial.print("Cell (");Serial.print(x);Serial.print(",");Serial.print(y);Serial.print(") minDistVecinos:");Serial.println(minDistance);
//                  Serial.println(distances[x][y]);
                  if (minDistance<distances[x][y])
                        distances[x][y]=minDistance+1;
                  visited[x][y]=1;
            }
            currCell=dequeueCell();
      }
}

void initializeMinDistances(){
      for(int i=0;i<16;i++)
            for(int j=0;j<16;j++){
                  distances[i][j]=1000;
                  visited[i][j]=0;
            }
      enqueuePointer=1;
      dequeuePointer=0;//empty queue
      distances[7][7]=0;
      enqueueCell(CellToStack(7,7));
      distances[7][8]=0;
      enqueueCell(CellToStack(7,8));
      distances[8][7]=0;
      enqueueCell(CellToStack(8,7));
      distances[8][8]=0;
      enqueueCell(CellToStack(8,8));

      updateDistancesWithQueue();

      for(int i=0;i<16;i++)
            for(int j=0;j<16;j++)
                  visited[i][j]=0;

}

void initializeDistancesIniGoal(){


      for(int i=0;i<16;i++)
            for(int j=0;j<16;j++){
                  distances[i][j]=1000;
                  visited[i][j]=0;
            }
      enqueuePointer=1;
      dequeuePointer=0;
      distances[0][0]=0;
      enqueueCell(CellToStack(0,0));

      updateDistancesWithQueue();

      for(int i=0;i<16;i++)
            for(int j=0;j<16;j++){
                  visited[i][j]=0;
            }
}


short yIncrementByDir(short dir){
      if(dir==UP_DIR)
            return 1;
      if(dir==DOWN_DIR)
            return -1;
      return 0;
}

short xIncrementByDir(short dir){
      if(dir==RIGHT_DIR)
            return 1;
      if(dir==LEFT_DIR)
            return -1;
      return 0;
}

short getNextMovement(short x,short y,short actDir){
      if(!hasWallInDir(walls[x][y],actDir) && distances[x+xIncrementByDir(actDir)][y+yIncrementByDir(actDir)] < distances[x][y])
            return STRAIGHT;
      short newDir = turnRightDir(actDir);
      if(!hasWallInDir(walls[x][y],newDir) && distances[x+xIncrementByDir(newDir)][y+yIncrementByDir(newDir)] < distances[x][y])
            return TURN_RIGHT;
      newDir = turnLeftDir(actDir);
      if(!hasWallInDir(walls[x][y],newDir) && distances[x+xIncrementByDir(newDir)][y+yIncrementByDir(newDir)] < distances[x][y])
            return TURN_LEFT;
      newDir = turn180(actDir);
      if(!hasWallInDir(walls[x][y],newDir) && distances[x+xIncrementByDir(newDir)][y+yIncrementByDir(newDir)] < distances[x][y])
            return BACK;

      return STOP;
}

short updateDir(short actDir, short mov){
      switch(mov){
            case STRAIGHT:
                  return actDir;
            case TURN_RIGHT:
                  return turnRightDir(actDir);
            case TURN_LEFT:
                  return turnLeftDir(actDir);
            case BACK:
                  return turn180(actDir);
            default:
                  return actDir;
      }
}
#ifdef SIMULATED_MAZE
void simulateFF(){
      short xPos=0;
      short yPos=0;
      short dir=UP_DIR;

      initializeDistancesCenterGoal();
      Serial.println("INITIAL MAZE:");
      printMaze(xPos,yPos,dir);

      while(distances[xPos][yPos]>0){
            long t=micros();
            updateCurrentWalls(xPos,yPos,wallsSim[xPos][yPos]);
            short cont = FFUpdateDistances(xPos,yPos);
            t=micros()-t;
//           Serial.print("Tiempo FF Upd: ");Serial.println(t);
//           Serial.print("Cont FF Upd: ");Serial.println(cont);
            printMaze(xPos,yPos,dir);
            short nextMovement = getNextMovement(xPos,yPos,dir);
            dir = updateDir(dir,nextMovement);

            yPos += yIncrementByDir(dir);
            xPos += xIncrementByDir(dir);
            //Serial.print("New pos: ");Serial.print(xPos);Serial.print(" ");Serial.print(yPos);Serial.print(" ");Serial.println(dir);
            //printMaze(xPos,yPos,dir);

            delay(300);
      }

      Serial.println("ARRIVED TO CENTER:");
      printMaze(xPos,yPos,dir);

      Serial.println("COME BACK:");
      initializeDistancesIniGoal();
      printMaze(xPos,yPos,dir);

      delay(1000);

      while(distances[xPos][yPos]>0){
            updateCurrentWalls(xPos,yPos,wallsSim[xPos][yPos]);
            short cont = FFUpdateDistances(xPos,yPos);
            short nextMovement = getNextMovement(xPos,yPos,dir);
            printMaze(xPos,yPos,dir);
            dir = updateDir(dir,nextMovement);

            yPos += yIncrementByDir(dir);
            xPos += xIncrementByDir(dir);
            //Serial.print("New pos: ");Serial.print(xPos);Serial.print(" ");Serial.print(yPos);Serial.print(" ");Serial.println(dir);
            //printMaze(xPos,yPos,dir);

            delay(300);
      }
      printMaze(xPos,yPos,dir);



      long t=micros();
      initializeMinDistances();
      t=micros()-t;
      Serial.print("Final MAZE (time ");Serial.print(t);Serial.println("us):");
      printMaze(xPos,yPos,dir);


}



void FFtestLoop(){
      delay(100);
      Serial.begin(115200);


      simulateFF();

      Serial.println("Save maze to EEPROM");
      printMaze(0,0,1);
      if(EEPROM_save_maze()>0)
            Serial.println("Saved");
      else
            Serial.println("EEPROM begin Error");

      delay(1000);

      if(EEPROM_load_maze()>0){
            Serial.println("Maze load from EEPROM:");
            printMaze(0,0,1);
      }

      while(1)delay(20000);
}
#endif



/*
void testQueue(){
      Serial.println(cellToQueue(2,5));
      for(i=0;i<16;i++)
            for(j=0;j<16;j++){
                  Serial.print(cellToQueue(i,j)); Serial.print(" ");
                  Serial.print(QueueToCellX(cellToQueue(i,j))); Serial.print(",");
                  Serial.print(QueueToCellY(cellToQueue(i,j))); Serial.println(" ");
            }
      for(i=1;i<10;i++)
            enqueueCell(i);
      for(j=1;j<7;j++)
            Serial.println(dequeueCell());
      for(;i<520;i++)
            enqueueCell(i);
      for(j=1;j<515;j++)
            Serial.println(dequeueCell());
}*/

void testStack(){
      Serial.println(CellToStack(2,5));
      int i,j;
      for(i=0;i<16;i++)
            for(j=0;j<16;j++){
                  Serial.print(CellToStack(i,j)); Serial.print(" ");
                  Serial.print(StackToCellX(CellToStack(i,j))); Serial.print(",");
                  Serial.print(StackToCellY(CellToStack(i,j))); Serial.println(" ");
            }
      for(i=1;i<10;i++)
            pushCell(i);
      for(j=1;j<7;j++)
            Serial.println(popCell());
      for(;i<520;i++)
            pushCell(i);
      for(j=1;j<515;j++)
            Serial.println(popCell());
}






