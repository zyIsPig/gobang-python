import numpy as np


import random
import time

random.seed(random.randint(1,100))

color_black = -1

color_white = 1

color_none = 0

opencontrol=False

five=100000;
four=40000;
blockfour=20000;
three=10000;
blockthree=100;
two=200;
blocktwo=10;
one=1;
blockone=1;
total=0;
maxsc=10*five;

full=False
list=[]
stepp=0
cnt=0
comtable=np.zeros(shape=(15,15),dtype=np.int64)
humtable=np.zeros(shape=(15,15),dtype=np.int64)

blackScore=np.zeros(shape=(15,15),dtype=np.int)
whiteScore=np.zeros(shape=(15,15),dtype=np.int)
blackScoreDir=np.zeros(shape=(15,15,4),dtype=np.int)
whiteScoreDir=np.zeros(shape=(15,15,4),dtype=np.int)

whiteTotal=0
blackTotal=0

movecnt=0
backcnt=0


starcut=0
abcut=0
cachecut=0


canget=[]

fourLongCount=0

maxscore=0

zyispig=[]

humcnt=0
comcnt=0

slide=15

ppp=np.zeros(shape=(15,15))
dir=[[1,0],[0,1],[1,1],[1,-1]]
leaf=0
init=0
map={}
chessb=np.zeros(shape=(15,15),dtype=np.int)

cacheget=1
time_out=False

ifinit=False

testtt=False

steppath=[]

aaa=1
ccc=1

starGo=False

testcontrol=False

ppp=False

killstarcnt=0

blackopen=0

# ran=random.randint(0,1)
ran=1
sta=time.time()
end=time.time()
c9=np.zeros(shape=(15,15))
c9[7,7]=-1
c9[6,6]=1
c9[8,7]=-1

c22=np.zeros(shape=(15,15))
c22[7,7]=-1
c22[7,8]=-1
c22[8,7]=-1
c22[9,7]=1
c22[8,8]=1
c22[7,9]=1
c22[10,6]=-1


c8=np.zeros(shape=(15,15),dtype=np.int)
c8[7,7]=-1
c8[6,6]=1
c8[6,8]=1
c8[6,7]=-1
c8[8,6]=-1

c7=np.zeros(shape=(15,15),dtype=np.int)
c7[7,7]=-1
c7[6,6]=1
c7[6,8]=-1
c7[7,6]=-1
c7[8,6]=1

# c6=np.zeros(shape=(15,15),dtype=np.int)
# c6[7,7]=-1
# c6[6,6]=1
# c6[7,5]=-1
c01=np.zeros(shape=(15,15))
c01[7,7]=-1
c01[8,6]=1
c01[6,6]=-1
c01[8,8]=1
c01[8,7]=-1
c01[9,7]=1
c01[10,6]=-1



c5=np.zeros(shape=(15,15),dtype=np.int)
c5[7,7]=-1
c5[7,8]=-1
c5[6,8]=-1
c5[8,7]=-1
c5[6,6]=1
c5[6,7]=1
c5[8,6]=1

c3=np.zeros(shape=(15,15),dtype=np.int)
c3[7,7]=-1
c3[6,6]=1
c3[8,6]=-1

c1=np.zeros(shape=(15,15),dtype=np.int)
c1[7,7]=-1
c1[6,6]=1
c1[8,6]=-1

c2=c1
c2[8,7]=1

c2[6,8]=-1
c2[5,9]=-1
c2[9,5]=1
c2[5,9]=-1
c2[4,10]=1
c2[5,8]=-1
c2[5,10]=1
c2[6,9]=-1
c2[6,10]=-1
c2[6,11]=1
c2[4,8]=1
c2[4,9]=-1

c4=np.zeros(shape=(15,15),dtype=np.int)
c4[7,7]=-1
c4[6,6]=1
c4[8,6]=-1
c4[6,8]=-1
c4[8,7]=1


c10=np.zeros(shape=(15,15),dtype=np.int)
c10[7,7]=-1
c10[8,6]=-1
c10[9,7]=-1
c10[6,8]=1
c10[8,8]=1
c10[10,8]=1


c11=np.zeros(shape=(15,15),dtype=np.int)
c11[7,7]=-1
c11[6,6]=-1
c11[5,7]=-1
c11[6,8]=1
c11[8,8]=1
c11[7,5]=1


c12=np.zeros(shape=(15,15),dtype=np.int)
c12[7,7]=-1
c12[6,6]=1
c12[8,6]=-1
c12[6,8]=1
c12[8,5]=-1
c12[7,8]=1
c12[7,5]=-1
c12[5,8]=1
c12[8,8]=-1
c12[8,7]=1
c12[9,7]=-1
c12[6,4]=1
c12[6,5]=-1

c13=np.zeros(shape=(15,15))
c13[7,7]=-1
c13[8,8]=-1
c13[9,9]=-1
c13[8,7]=1
c13[7,8]=1
c13[10,10]=1

c14=np.zeros(shape=(15,15))
c14[7,7]=-1
c14[6,6]=1
c14[6,8]=1
c14[8,6]=-1
c14[8,5]=-1
c14[7,8]=1
c14[6,5]=-1


c15=np.zeros(shape=(15,15))
c15[7,7]=-1
c15[6,6]=1
c15[8,7]=-1
c15[7,5]=1
c15[8,6]=-1

c16=np.zeros(shape=(15,15))
c16[7,7]=-1
c16[8,8]=-1
c16[9,9]=-1
c16[8,7]=1
c16[9,8]=1
c16[10,10]=1
# c14[7,8]=1
# c14[6,5]=-1
c17=np.zeros(shape=(15,15))
c17[7,7]=-1
c17[8,6]=-1
c17[8,7]=1
c17[8,8]=1
# c12[5,5]=1
# c12[9,5]=-1
# c12[10,5]=1
# c12[9,6]=-1
c18=np.zeros(shape=(15,15))
c18[7,7]=-1


c19=np.zeros(shape=(15,15))
c19[7,7]=-1
c19[8,8]=1
c19[6,6]=-1
c19[6,8]=1
c19[7,8]=-1
c19[7,9]=1
c19[6,10]=-1


c20=np.zeros(shape=(15,15))
c20[7,7]=-1
c20[8,8]=-1
c20[7,8]=1
c20[9,7]=1

c21=np.zeros(shape=(15,15))
c21[7,6]=-1
c21[6,7]=-1
c21[6,8]=-1
c21[8,7]=-1
c21[7,7]=1
c21[8,6]=1
c21[5,8]=1
c21[9,8]=1


c23=np.zeros(shape=(15,15))
c23[6,7]=-1
c23[7,6]=-1
c23[7,8]=-1
c23[8,8]=-1
c23[8,9]=-1
c23[8,7]=1
c23[8,5]=1
c23[8,4]=1
c23[7,5]=1
c23[9,10]=1


c24=np.zeros(shape=(15,15))
c24[7,6]=-1
c24[6,7]=-1
c24[8,7]=-1
c24[5,8]=1
c24[7,8]=1
c24[9,8]=1


c25=np.zeros(shape=(15,15))
c25[6,7]=-1
c25[5,6]=-1
c25[7,8]=-1
c25[7,7]=1
c25[6,8]=1
c25[8,9]=1


c6=np.zeros(shape=(15,15))
c6[7,7]=-1
c6[6,8]=1
c6[8,8]=-1
c6[6,6]=1
c6[8,9]=-1
c6[8,6]=1
c6[7,9]=-1
c6[7,6]=1
c6[9,6]=-1
c6[6,9]=1
c6[6,7]=-1


c00=np.zeros(shape=(15,15))
c00[7,7]=-1
c00[6,8]=1
c00[8,8]=-1
c00[6,6]=1
c00[8,9]=-1
c00[8,6]=1
c00[7,9]=-1
c00[7,6]=1
c00[9,6]=-1
c00[6,9]=1
c00[6,7]=-1
c00[9,7]=1
c00[10,8]=-1
# c6[6,11]=1
# 5,9,-1
c02=np.zeros(shape=(15,15))

c02[7,7]=-1
c02[8,6]=1
c02[6,6]=-1
c02[8,8]=1
c02[6,5]=-1
c02[6,8]=1
c02[7,5]=-1
c02[7,8]=1
c02[5,8]=-1
c02[8,5]=1
c02[8,7]=-1
c02[9,8]=1
c02[10,8]=-1
c02[5,7]=1
c02[4,6]=-1



type1=np.zeros(shape=(15,15))

pvs=False

attackpath=[]

defenceAndAttack=[]
control=False

defencepath=[]

record=[]
path=[]
defenceattackCount=1

testa=0
testb=0
errcount=0


xie=False
heng=False




def open(x,y):


    if abs(x-7)<=1 and abs(y-7)<=1:
        return True
    return False


def whiteop(chessboard,x,y,len,num,color):
    for i in range(x-len,x+len+1):
        if i<0 or i>=15:
            continue
        for j in range(y-len,y+len+1):
            if j<0 or j>=15:
                continue
            if i==x and j==y:
                continue
            if chessboard[i,j]==color:
                num-=1
                if num==0:
                    return True
    return False



def blackopen(x,y,i,j):
    if abs(x-i)==1 and abs(y-j)==1:
        return True
    return False
# def zzzzz(chessboard):
#     for i in range(15):
#         for j in range(15):
#             if chessboard[i,j]==


def bagua(chessboard,x,y,colr):

    for i in range(x-2,x+3):
        if i < 0 or i >= 15:
            continue
        for j in range(y-2,y+3):
            if j<0 or j>=15:

                continue

            if abs(x-i)==2 and abs(y-j)==1 and chessboard[i,j]==colr:
                return True
            if abs(x-i)==1 and abs(y-j)==2 and chessboard[i,j]==colr:
                return True
    return False





def getusergo(chessboard):

    for i in range(15):
        for j in range(15):
            if chessboard[i,j]!=chessb[i,j]:
                return [i,j]
    return False

def zabristinit():
    global init

    print("initial success")

    # cacheget=0
    # map={}
    global init
    init=np.random.randint(1,1000000000000)
    for i in range(15):
        for j in range(15):
            comtable[i,j]=np.random.randint(1,1000000000000)
            humtable[i,j]=np.random.randint(1,1000000000000)


def zabmove(x,y,com):
    global init
    if com==-1:
        init=init^comtable[x,y]
    else:
        init=init^humtable[x,y]


def save(score,depth):
    global init
    map[init]=[score,depth]

def check():
    if init in map:
        return map[init]
    else:
        return False


def get_user_move(chessboard):

    for i in range(15):
        for j in range (15):

            if chessboard[i,j]!=chessb[i,j]:
                return [i,j]


def updateScore(chessboard,x,y):
    # global whiteTotal,blackTotal

    color = chessboard[x,y]

    if color == -1:
        # blackTotal -= blackScore[x,y]
        # blackScore[x,y]=calculate4(x,y,chessboard,-1,0)

        # blackTotal += blackScore[x,y]
        blackScore[x,y]=0

        # whiteTotal -= whiteScore[x,y]
        whiteScore[x,y]=0


    if color == 1:
        # whiteTotal -= whiteScore[x,y]
        # whiteScore[x,y]=calculate4(x,y,chessboard,1,0)
        whiteScore[x,y]=0
        # whiteTotal += whiteScore[x,y]

        # blackTotal -= blackScore[x,y]
        blackScore[x][y]=0

    if color == 0:
        # blackTotal -= blackScore[x,y]
        blackScore[x,y]=calculate4(x,y,chessboard,-1,0)


        # blackTotal += blackScore[x,y]


        # whiteTotal -= whiteScore[x,y]
        whiteScore[x,y]=calculate4(x,y,chessboard,1,0)


        # whiteTotal += whiteScore[x,y]












    for i in range(4):
        tempx=dir[i][0]

        tempy = dir[i][1]


        tempi = x + tempx

        temj = y + tempy

        count = 5

        while count != 0:
            if tempi < 0 or tempi >= 15 or temj < 0 or temj >= 15:
                break


            updateApoint(tempi, temj, i,chessboard)
            tempi += tempx
            temj += tempy
            count=count-1


        tempi = x - tempx
        temj = y - tempy
        count = 5

        while count != 0:
            if tempi < 0 or tempi >= 15 or temj < 0 or temj >= 15:
                break


            updateApoint(tempi, temj, i,chessboard)
            tempi -= tempx
            temj -= tempy
            count=count-1





def updateApoint(x,y,dir,chessboard):


    role = chessboard[x,y]

    if role == -1:
        # blackScore[x,y] -= blackScoreDir[x,y,dir]
        # blackTotal=blackTotal-blackScoreDir[x,y,dir]
        blackScoreDir[x,y,dir]=calculate(x,y,chessboard,dir,-1,0)
        blackScore[x,y]=0


        # blackScore[x,y] += blackScoreDir[x,y,dir]
        # blackTotal += blackScoreDir[x,y,dir]




    elif role == 1:
        # whiteScore[x,y] -= whiteScoreDir[x,y,dir]
        # whiteTotal=whiteTotal-whiteScoreDir[x,y,dir]
        whiteScoreDir[x,y,dir]=calculate(x,y,chessboard,dir,1,0)

        # whiteScore[x,y] += whiteScoreDir[x,y,dir]
        whiteScore[x,y]=0
        # whiteTotal += whiteScoreDir[x,y,dir]


    else :
        blackScore[x,y] -= blackScoreDir[x,y,dir]
        # blackTotal = blackTotal - blackScoreDir[x,y,dir]




        blackScoreDir[x,y,dir]=calculate(x,y,chessboard,dir,-1,0)
        blackScore[x,y] += blackScoreDir[x,y,dir]
        # blackTotal += blackScoreDir[x,y,dir]

        whiteScore[x,y] -= whiteScoreDir[x,y,dir]
        # whiteTotal = whiteTotal - whiteScoreDir[x,y,dir]

        whiteScoreDir[x,y,dir]=calculate(x,y,chessboard,dir,1,0)
        whiteScore[x,y] += whiteScoreDir[x,y,dir]
        # whiteTotal += whiteScoreDir[x,y,dir]


def scoreinit(chessboard):
    # print("pppppppppppp")
    for k in range(4):
        for i in range(15):
            for j in range(15):
                updateApoint(i,j,k,chessboard)


def calculate4(x,y,chessboard,color,pattern):
    total=0
    for i in range(4):
        total+=calculate(x,y,chessboard,i,color,pattern)



    return total


def calculate(x,y,chessboard,dir_control,color,partten):
    count=1
    empty=0

    block=0
    model=""

    dirtemp=dir[dir_control]
    tempx=x
    tempy=y


    while 1:
        if tempx+dirtemp[0] >= 15 or tempy+dirtemp[1]>=15  or tempy+dirtemp[1]<0:
            block += 1
            model.join('-1')
            break
        if chessboard[tempx+dirtemp[0],tempy+dirtemp[1]] == -color :
            block += 1
            model.join('-1')
            break

        if chessboard[tempx+dirtemp[0],tempy+dirtemp[1]]==color:
            count+=1
            tempx+=dirtemp[0]
            tempy+=dirtemp[1]
            model=model+'1'
        elif chessboard[tempx+dirtemp[0],tempy+dirtemp[1]] ==0:
            if empty==0 and tempx+2*dirtemp[0]<15 and tempy+2*dirtemp[1]<15 and tempy+2*dirtemp[1]>=0 and chessboard[tempx+2*dirtemp[0],tempy+2*dirtemp[1]]==color:
                empty=count+1
                model=model+'0'
                tempx+=dirtemp[0]
                tempy+=dirtemp[1]
            else:
                break



    tempx=x
    tempy=y

    while 1:
        # print("pig")
        if tempx - dirtemp[0] ==-1 or tempy - dirtemp[1] == 15 or tempy - dirtemp[1] ==-1:
            block += 1
            model='-1'+model
            break
        if chessboard[tempx - dirtemp[0],tempy - dirtemp[1]] == -color:
            block += 1
            model ='-1'+model
            break

        if chessboard[tempx - dirtemp[0],tempy - dirtemp[1]] == color:
            count += 1
            if empty!=0:
                empty+=1
            tempx -= dirtemp[0]
            tempy -= dirtemp[1]
            model = '1'+model
        elif chessboard[tempx - dirtemp[0],tempy - dirtemp[1]] == 0:
            if empty==0 and tempx - 2 * dirtemp[0] >=0 and tempy - 2 * dirtemp[1] < 15 and tempy - 2 * dirtemp[1] >= 0  and chessboard[tempx-2*dirtemp[0],tempy-2*dirtemp[1]]==color:
                empty=1
                model = model + '0'
                tempx -= dirtemp[0]
                tempy -= dirtemp[1]
            else:
                break

    if partten==1:
        count-=1


    scoretemp=get_score(model,count,block,empty)
    return scoretemp



def get_score(model,count,block,empty):

    if block==0:
        if empty==0:
            if count>=5:
                return five

            map={5:five,4:four,3:three,2:two,1:one}

            return map[count]
            # if count >=5:
            #     return five
            # if count ==4:
            #     return four
            # if count ==3:
            #     return three
            # if count ==2:
            #     return two
            # if count ==1:
            #     return one
        if empty==2 or empty ==count:
            if count >=6:
                return five
            map={5:four,4:blockfour,3:three,2:two}
            return map[count]
            # if count ==5:
            #     return four
            # if count ==4:
            #     # B
            #     return blockfour
            # if count ==3:
            #     # B
            #     return three
            # if count ==2:
            #     # B
            #     return two
        if empty ==3 or empty==count-1:
            if count >=7:
                return five
            # if count ==6:
            #     return four
            # if count ==5:
            #     return blockfour
            # if count ==4:
            #     return blockfour
            map={6:four,5:blockfour,4:blockfour}
            return map[count]


        if empty==4 or empty==count-2:
            if count >=8:
                return five
            # if count ==7:
            #     return four
            # if count==6:
            #     return blockfour
            map={7:four,6:blockfour}
            return map[count]

        if empty==5 or empty==count -3:
            if count >= 9:
                return five
            if count==8:
                return four

    if block ==1:
        if empty==0:
            if count>=5 :
                return five
            # if count ==4:
            #     return blockfour
            # if count ==3:
            #     return blockthree
            # if count==2:
            #     return blocktwo
            # if count ==1:
            #     return blockone
            map = {4: blockfour,3:blockthree,2:blocktwo,1:blockone}
            return map[count]

        if empty==2 or empty==count:
            if count>=6:
                return five
            if count==5:
                if model=="-1101111" or model=="111101-1":
                    return four
                else:
                    # B

                    return blockfour
            # if count==4:
            #     # if model=="-110111" or model=="11101-1":
            #     #     # B
            #     #     return blockfour
            #     # else:
            #     return blockfour
            # if count ==3:
            #     return blockthree
            # if count ==2:
            #     return blocktwo

            map = {4: blockfour, 3: blockthree,2:blocktwo}
            return map[count]
        if empty==3 or empty==count-1:

            if count >=7:
                return five
            if count ==6 :
                if model=="-11101111" or model=="1111011-1":
                    return four
                else:
                    return blockfour
            if count ==5:
                # if model=="-1110111" or model=="111011-1":
                #     # B
                #     return blockfour
                # else:
                return blockfour
            if count ==4:
                # C
                return blockfour
        if empty==4 or empty==count-2:
            if count >=8:
                return five
            if count ==7:
                if model=="-111101111" or model=="11110111-1":
                    return four
                else:
                    # B
                    return blockfour
            if count ==6:
                # BB
                return blockfour

        if empty==5 or empty==count-3:
            if count>=9:
                return five
            if count==8:
                return four

    if block==2:
        if empty==0:
            if count>=5:
                return five
            else:
                return 0

        if empty==2 or empty==count:
            if count >=6:
                return five
            if count ==5 or count ==4:
                return blockfour

            else:
                return 0
        if empty==3 or empty==count-1:
            if count >=7:
                return five
            # if count==6 or count==5:
            #     return blockfour
            # if count ==5:
            #     return blockfour
            # if count==4:
            #     return blockfour
            else:
                return blockfour

        if empty==4 or empty==count-2:
            if count >=8:
                return five
            if count==7 or count==6:
                return blockfour


        if empty==5 or empty==count-3:
            if count>=9:
                return five
            if count ==8:


                return blockfour

    return 0
def max(a,b):
    if a>b:
        return a
    else:
        return b
def hasnei(chessboard,x,y,len,num):
    for i in range(x-len,x+len+1):
        if i<0 or i>=15:
            continue
        for j in range(y-len,y+len+1):
            if j<0 or j>=15:
                continue
            if i==x and j==y:
                continue
            if chessboard[i,j]!=0:
                num-=1
                if num==0:
                    return True
    return False


def gettotal(chessboard,color):
    score=0
    for i in range(15):
        for j in range(15):
            if chessboard[i,j]!=0:
                continue
            if cnt<6 and hasnei(chessboard,i,j,1,1)==False:
                continue
            if cnt>=6 and hasnei(chessboard,i,j,2,2)==False:
                continue;
            score+=calculate4(i,j,chessboard,color,0)

    return score









def takenthird(a):
    return a[2]



def genopen(chessboard,com):

    if com==-1:
        if cnt==0:
            return []
        if cnt==1:
            # return [[7,6,0],[6,7,0],[8,7,0,[7,8,0]]]
            return [[6,6,0],[8,8,0],[8,6,0],[6,8,0]]
        if cnt==2 :
            # l=[[6,5,0],[5,6,0],[8,5,0],[9,6,0],[9,8,0],[8,9,0],[5,8,0],[6,9,0]]
            # l=[[5,5,0],[9,5,0],[5,9,0],[9,9,0],[8,6,0],[6,8,0],[6,6,0],[8,8,0]]
            l=[[5,7,0],[7,5,0],[7,9,0],[9,7,0]]
            # l=[[6,7,0],[7,6,0],[8,7,0],[7,8,0]]
            return l
            # for i in l:
            #     if chessboard[i[0],i[1]]!=0:
            #         continue
            #     if whiteop(chessboard,i[0],i[1],1,1,-1):
            #         return [i]
        #     p=[[5,7,0],[7,9,0],[9,7,0],[7,5,0]]
        #
        #     for i in p:
        #         if whiteop(chessboard,i[0],i[1],1,1,-1):
        #             return [i]
    # if com==1:
    #     for




def gen(chessboard,com):
    comfivetemp = []
    comfourtemp = []
    comblockfourtemp = []
    comtwothreetemp = []
    comthreetemp = []
    comtwotemp = []

    humfivetemp = []
    humfourtemp = []
    humblockfourtemp = []
    humtwothreetemp = []
    humthreetemp = []
    humtwotemp = []
    othertemp = []

    attack = []
    defent = []

    potentialPoint = []

    humtwotwotemp=[]
    # combigpot=[]
    # humbigpot=[]
    #
    # comtemp = 0
    # humtemp = 0
    if starGo:
        for i in range(steppath.__len__()-1,-1,-2):
            point = steppath[i]
            co = chessboard[point[0],point[1]]
            p = calculate4(point[0],point[1],chessboard,co,0)
            if p > three:
                attack.append([point[0], point[1]])
                break



        for i in range(steppath.__len__()-2,-1,-2):
            point = steppath[i]
            p = calculate4(point[0],point[1],chessboard,chessboard[point[0],point[1]],0)
            if p > three:
                defent.append([point[0], point[1]])
                break


        if attack.__len__() == 0:
            point = steppath[steppath.__len__() - 1]
            attack.append([point[0], point[1]])

        if defent.__len__() == 0:
            point = steppath[steppath.__len__() - 2]

            defent.append([point[0], point[1]])



    for i in range(15):
        for j in range(15):
    # for i,j in enumerate(chessboard):

            if chessboard[i][j]!= 0:


                continue

            # if not testcontrol and com==-1 and cnt <2 and not blackopen(i,j,7,7):
            #     continue

            # if not testcontrol and com==1 and cnt==2 and not whiteop(chessboard,i,j,1,2,-1):
            #     continue
            # if not testcontrol and com==1 and cnt <3 and  not blackopen(i,j,7,7):
            #         continue

            # if not testcontrol and com==1 and cnt>1 and cnt <=3 and not bagua(chessboard,i,j,1):
            #     continue
            # if not testcontrol and com == 1 and cnt <8 and cnt >=3 and not whiteop(chessboard,i, j, 1, 2, -1):
            #     continue
            # if not testcontrol and com==1 and cnt<=7 and
            # if not testcontrol and com==1 and cnt <=8 and not whiteop(chessboard,i,j,1,1,-1):
            #     continue
            # if not testcontrol and com==-1 and comcnt<3:
            #     if cnt==0:
            #         return [7,7]
            #     if cnt==1:
            #         if not blackopen(i,j,7,7):
            #             continue
            #     if cnt==2:
            # if com==1 and cnt<=9 and not hasnei(chessboard,i,j,1,1) :
            #     continue


            if cnt >=6  and not hasnei(chessboard,i,j,2,2):

                continue

            # if cnt <=3 and not hasnei(chessboard,i,j,1,2):
            #     continue
            if cnt <6 and not hasnei(chessboard,i,j,1,1):
                continue


            # comtemp = calculate4(i,j,chessboard,com,0)
            # humtemp = calculate4(i,j,chessboard,-com,0)

            #
            if com == -1:
                comtemp=blackScore[i,j]
                humtemp=whiteScore[i,j]
            else:
                comtemp=whiteScore[i,j]
                humtemp=blackScore[i,j]


            maxtemp = max(comtemp,humtemp)
            # global maxscore
            # if maxtemp > maxscore:
            #     maxscore = maxtemp


            # if not testcontrol and cnt <3 and not whiteop(chessboard,i,j,1,1,-1):
            #     continue
            # if maxtemp<four and not testcontrol and cnt>=3 and cnt <=8 and not whiteop(chessboard,i,j,2,2,-1):
            #     continue

            if starGo:
                a = starCheck(attack[attack.__len__() - 1], [i, j])
                b = starCheck(defent[defent.__len__() - 1], [i, j])

                if maxtemp >= blockfour:
                    pass
                elif ((starGo) and (not a) and ( not b)):
                    # global starcut
                    # starcut+=1
                    continue


            if comtemp >= five:
                comfivetemp.append([i, j, comtemp])

            elif humtemp >= five:
                humfivetemp.append([i, j, -humtemp])

            elif comtemp >= four:
                comfourtemp.append([i, j, comtemp])

            elif humtemp >= four:
                humfourtemp.append([i, j, -humtemp])
            elif comtemp >= blockfour:
                # if cnt<6:
                #     pass
                # else:
                    comblockfourtemp.append([i, j, comtemp])
            elif humtemp >= blockfour:
                humblockfourtemp.append([i, j, -humtemp])
            elif (comtemp >= 2*three):
                comtwothreetemp.append([i, j, comtemp])
            elif (humtemp >= 2*three):
                humtwothreetemp.append([i, j, -humtemp])

            elif comtemp >= three:
                comthreetemp.append([i, j, comtemp])
            elif humtemp >= three:
                humthreetemp.append([i, j, -humtemp])
            elif comtemp >= 2 * two:
                potentialPoint.append([i, j, comtemp])
            elif humtemp>=2*two:
                humtwotwotemp.append([i,j,humtemp])
            elif comtemp >= two:
                comtwotemp.append([i, j, comtemp])

            elif (humtemp >= two):
                humtwotemp.append([i, j, humtemp])
            else :
                othertemp.append([i, j, maxtemp])






    if comfivetemp.__len__() >= 1:

        return comfivetemp

    if humfivetemp.__len__() >= 1:
        return humfivetemp

    if comfourtemp.__len__() >= 1:
        return comfourtemp
    if humfourtemp.__len__() >= 1 and comblockfourtemp.__len__() >= 1:

        return humfourtemp+comblockfourtemp

    if humfourtemp.__len__() >= 1 and comblockfourtemp.__len__() < 1:

        return humfourtemp


    if comtwothreetemp.__len__() >= 1:
        return comtwothreetemp+(comblockfourtemp)+(humblockfourtemp)

    if humtwothreetemp.__len__() >= 1:
        return humtwothreetemp+(comblockfourtemp)+comthreetemp


    if comblockfourtemp.__len__() >= 1 or comthreetemp.__len__() >= 1 or humblockfourtemp.__len__() >= 1 or humthreetemp.__len__() >= 1:
        # if com==-1 and potentialPoint.__len__()>=1 and cnt <=5:
        #     return combigpot+potentialPoint+comthreetemp
        # if cnt<=5:
        #     return comthreetemp+humthreetemp+combigpot+humbigpot+potentialPoint+comblockfourtemp+humblockfourtemp
        # else:
        # if com==1 and cnt<=6:
        #     return humthreetemp+humblockfourtemp+humtwotwotemp+comthreetemp+potentialPoint+comtwotemp
        if com==-1 and cnt<=7:
            return comthreetemp +comblockfourtemp+potentialPoint+comtwotemp

        return comblockfourtemp+comthreetemp+humblockfourtemp+humthreetemp

    # if potentialPoint.__len__()>=1 and com==-1:
    #     return potentialPoint
    # if potentialPoint.__len__()>=1 or humtwotwotemp.__len__()>=1:
    #     return potentialPoint+humtwotwotemp
    # else:
    t =potentialPoint+humtwotwotemp+ comtwotemp+humtwotemp+othertemp

    t.sort(key=lambda a: a[2], reverse=True)

    return t




def move(chessboard,x,y,color):
    # global cnt,movecnt
    chessboard[x,y]=color

    updateScore(chessboard,x,y)
    zabmove(x,y,color)
    steppath.append([x,y,color])
    # cnt+=1
    # movecnt+=1

def back(chessboard,x,y):
    # global cnt,backcnt
    zabmove(x,y,chessboard[x,y])
    chessboard[x,y]=0
    updateScore(chessboard,x,y)
    steppath.pop()
    # cnt-=1
    # backcnt+=1











def max_(depth,com,alpha,beta,totalDepth,chessboard,spread,one,two):

    if time.time()-sta>=4.7:
        return -9999999
    m=-maxsc


    if depth!=totalDepth:
        aa = check()
        if aa != False:
            if aa[1] >= depth:
                # global cachecut
                # cachecut += 1
                # print("pp")
                return aa[0]

    global starGo
    if one>=1 and two>=1:
        starGo=True
    else:
        starGo=False

    # if not testcontrol and com==-1 and cnt<90:
    #     starGo=True







    if depth==0:
        # blackTotal=gettotal(chessboard,-1)
        # whiteTotal=gettotal(chessboard,1)
        blackTotal=np.sum(blackScore)
        whiteTotal=np.sum(whiteScore)
        #
        # global leaf
        # leaf+=1
        if com==-1:
            return blackTotal-whiteTotal
        else:
            return whiteTotal-blackTotal


    if cnt==3 and com==-1 and depth==totalDepth and ran==1:
        candidate=black(chessboard)
    else:
        candidate=gen(chessboard,com)







    for i in range(candidate.__len__()):

        tempi=candidate[i][0]
        tempj=candidate[i][1]

        if chessboard[tempi,tempj]!=0:
            continue
        # if candidate[i].__le__()<2:
        #     s=0
        # else:
        s=candidate[i][2]


        # if depth==totalDepth:
        #     zyispig.append([tempi,tempj,s])
        if i==0 and depth==totalDepth:
            list.append([tempi,tempj,0])
        if s>=five and depth!=totalDepth:
            return maxsc

        if  s <= -five and spread < fourLongCount:

            depth += 2
            spread+=1
            totalDepth += 2






        # global testa
        # testa=init
        move(chessboard,tempi,tempj,com)

        if i==0 or pvs:
            sc=min_(depth - 1, -com, alpha, beta, totalDepth, chessboard,spread,one,two+1)
        else:
            sc=min_(depth - 1, -com, alpha, alpha+1, totalDepth, chessboard,spread,one,two+1)

            if sc>alpha and sc<beta:

                sc = min_(depth - 1, -com, sc, beta, totalDepth, chessboard,spread,one,two+1)



        back(chessboard,tempi,tempj)
        # testb=init

        # if(testa!=testb):
        #     print("zyispig")
        #     global errcount
        #     errcount+=1
        #     # pass
        if sc>m:
            m=sc



        if  depth==totalDepth:

            # list.append([tempi,tempj,sc])
            pass

        if sc>alpha:
            alpha=sc
            if  depth==totalDepth:
                list.append([tempi, tempj,sc])


        if alpha>=beta:
            # global abcut
            # abcut+=1
            break




    save(m,depth)

    return m


def min_(depth,com,alpha,beta,totalDepth,chessboard,spread,one,two):




    aa = check()
    if aa != False:
        if aa[1] >= depth:
            # global cachecut
            # cachecut += 1
                # print("pp")
            return aa[0]

    m=maxsc
    global starGo
    if one>=1 and two>=1:
        starGo=True
    else:
        starGo=False

    candidate = gen(chessboard,com)


    for i in range(candidate.__len__()):
        tempi=candidate[i][0]
        tempj=candidate[i][1]
        s=candidate[i][2]

        if  s>=five:
            return -maxsc

        if s <= -five and spread < fourLongCount:
            depth += 2
            totalDepth += 2
            spread+=1


        move(chessboard,tempi,tempj,com)

        if i==0 or pvs:

            sc = max_(depth - 1, -com, alpha, beta, totalDepth, chessboard,spread,one+1,two)
        else:
            sc = max_(depth - 1, -com, beta-1, beta, totalDepth, chessboard,spread,one+1,two)

            if sc>alpha and sc<beta:
                sc= max_(depth - 1, -com, alpha, sc, totalDepth, chessboard,spread,one+1,two)


        back(chessboard,tempi,tempj)


        if  sc<m:
            m=sc


        if  sc<beta:
            beta=sc

        if  alpha>=beta:
            # global abcut
            # abcut+=1
            break



    save(m,depth)

    return m



def it(depth,chessboard,com,alpha,beta,):



   for i in range(2,8):
       max_(i,com,alpha,beta,i,chessboard,0,0,0)
       if list.__len__()==0:
           return
       point=list[list.__len__()-1]
       if point[2]>=five or point[2]<=-five:
           return



#













def test12():
    chessboard=np.zeros(shape=(15,15))






def test11():
    chessboard = np.zeros((15,15), dtype=np.int)
    chessboard[1, 3] = 1
    chessboard[2, 2] = 1
    chessboard[2, 5] = 1
    chessboard[3:5, 3] = 1
    chessboard[1, 11:13] = -1
    chessboard[2, 11:13] = -1
    chessboard[5, 13] = -1

    print(chessboard)

    aigo(chessboard,-1)

    print(list)




def test():
    chessboard=np.zeros(shape=(15,15),dtype=np.int)
    chessboard[2, 2] = 1
    chessboard[3, 3] = 1
    chessboard[4, 4] = 1
    chessboard[5, 6] = 1
    chessboard[5, 8] = 1
    chessboard[1:3, 11] = -1
    chessboard[3, 9:11] = -1
    chessboard[6, 13] = -1
    print(chessboard)
    aigo(chessboard,-1)
    # pvsnegmax(chessboard,-1,2,2)

    a=list[list.__len__()-1][0]
    b=list[list.__len__()-1][1]

    print([a,b])

def test1():
    chessboard = np.zeros(shape=(15, 15))
    ttt=np.zeros(shape=(15,15))
    # chessboard = np.zeros((self.chessboard_size, self.chessboard_size), dtype=np.int)
    chessboard[2, 2:4] = 1
    chessboard[4, 1:3] = 1
    chessboard[1, 10:12] = -1
    chessboard[2, 10] = -1
    chessboard[4, 12] = -1
    # chessboard[1,9]=-1
    # chessboard[1,8]=1
    # chessboard[3,11]=-1
    # chessboard[5,13]=1
    print(chessboard)
    scoreinit(chessboard)

    print(blackScore)
    print(whiteScore)
    comscore=gettotal(chessboard,-1)
    humscore=gettotal(chessboard,1)
    print(comscore)
    print(humscore)
    comscore=np.zeros(shape=(15,15))

    # for i in range(15):
    #     for j in range(15):
    #         comscore[i][j]=calculate4(i,j,chessboard,-1)
    #         # print(int(comscore[i][j]),end="")
    #         # print("  ",end="")
    #
    #     # print()

    # print(comscore)
    # assert (comscore)
    # max_(chessboard,4,4,-999999,999999,-1,-1)

    # p=vcx(-1,10,chessboard)
    # print(p)
    aigo(chessboard,-1)
    if list.__len__()==0:
        list.append([7,7])

    a=list[list.__len__()-1][0]
    b=list[list.__len__()-1][1]

    print([a,b])


def getcount(chessboard,color):
    global cnt,humcnt,comcnt


    for i in range(15):
        for j in range(15):
            if chessboard[i,j]!=0:
                cnt = cnt +1
                if chessboard[i,j]==color:
                    comcnt+=1
                else:
                    humcnt+=1
    # print(cnt)




def test3():
    chessboard = np.zeros(shape=(15, 15))
    ttt = np.zeros(shape=(15, 15))
    chessboard[5][7]=-1
    chessboard[6:9,7]=1
    chessboard[9,8]=1
    chessboard[9,9]=1

    chessboard[3:6,4]=-1
    chessboard[6,2:4]=-1
    # chessboard[8][7]=1
    # chessboard[7][7]=1
    print(chessboard)
    aigo(chessboard,1)

    # pvsnegmax(chessboard, -1, 2, 2, -999999, 999999)

    a = list[list.__len__() - 1][0]
    b = list[list.__len__() - 1][1]

    print([a, b])
def test4():
    chessboard = np.zeros(shape=(15, 15))
    comscore = np.zeros(shape=(15, 15))

    chessboard[6:9,6]=-1
    chessboard[5,6]=1

    for i in range(15):
        for j in range(15):
            if chessboard[i,j]!=0:
                continue
            comscore[i][j]=calculate4(i,j,chessboard,1)
            # print(int(comscore[i][j]),end="")
            # print("  ",end="")

        # print()

    print(comscore)


    print(chessboard)




    # for i in range(15):
    #     for j in range(15):
    #         ttt[i][j]=calculate4(i,j,chessboard,1)
    #         # print(int(comscore[i][j]),end="")
    #          # print("  ",end="")
    # print(ttt)

def test6():
    chessboard=np.zeros(shape=(15,15))
    chessboard[7,5:9]=-1
    chessboard[8,5:9]=1
    aigo(chessboard,1)
    print(chessboard)
    print("ring")
    print(list)
def play():
    chessboard = np.zeros(shape=(15, 15))
    global leaf,cnt
    leaf=0

    while True:
        aigo(chessboard, -1)
        a = list[list.__len__() - 1][0]
        b = list[list.__len__() - 1][1]
        print([a, b])
        # playergo(x,y,chessboard)
        chessboard[a, b] = -1
        x = int(input("x"))
        y = int(input("y"))
        chessboard[x, y] = 1


        # playergo(x,y,chessboard)




        # x = int(input("x"))
        # y = int(input("y"))
        chessboard[x,y]=1

        # steppath.append([x,y])
        # move(chessboard, a, b, 1)
        # zabmove(a,b,1)




        # cnt+=1
        # cnt-=1
        print(cnt)
        # zabmove(x,y,-1)



        # print(leaf)
        # leaf=0



        # print(whiteScore)
        # print(chessboard)
        # pvsnegmax(chessboard,1,4,4,-9999999,9999999)
        #
        print(steppath)

        print(chessboard)

def playergo(x,y,chessboard):

    chessboard[x][y]=-1



def test5():
    chessboard = np.zeros(shape=(15, 15))
    chessboard[1:4,1]=1
    chessboard[0:4,0]=-1
    print(chessboard)

    aigo(chessboard,1)

    print(list)

def test7():
    chessboard = np.zeros(shape=(15, 15))
    chessboard[7,6:9]=-1
    chessboard[8:10,7]=-1
    print(chessboard)
    aigo(chessboard,1)
    print(list)




def lll(chessboard,com):

    scorecom=0
    scorehum=0

    for i in range(15):
        for j in range(15):
            scorecom=gettotal(chessboard,com)
            scorehum=gettotal(chessboard,-com)

            temp=scorecom-scorehum


def aigo(chessboard,color):

    global sta,ccc,time_out,ifinit,testtt,cachecut,abcut,leaf,starcut,cnt
    cachecut=0
    abcut=0
    starcut=0
    leaf=0
    time_out=False
    global p
    p=0

    if ifinit == False:
        # print("paopopcknckn")
        print("now we are in pattern %d"%(ran))
        scoreinit(chessboard)
        getcount(chessboard,color)
        if cnt>=4:
            global testcontrol
            testcontrol=True
        zabristinit()
        ifinit = True
        global starGo
        starGo=True

    sta=time.time()

    global cacheget
    # print(chessboard)


    # zabristinit()
    # print(cnt)

    newstep = getusergo(chessboard)
    if newstep!=False:
        zabmove(newstep[0],newstep[1],-color)
        updateScore(chessboard,newstep[0],newstep[1])
        global chessb,humcnt
        chessb[newstep[0],newstep[1]]=-color
        steppath.append(newstep)
        humcnt+=1

    # if testtt == True and newstep[0] == 1 and newstep[1] == 1:
    #     print("alskal")
    #     list.append([14, 14])



    # if ccc==6:
    #

    # max_(4, color, -999999999, 999999999, 4, chessboard, 0,0,0)
    print("cnt%d"%(cnt))


    # it(6,chessboard,-1,-999999999,999999999)

    # max_(4, color, -999999999, 999999999, 4, chessboard, 0, 0, 0)
    #
    # if cnt<=8:
    #     global opencontrol
    #     opencontrol=True
    #     max_(4, color, -999999999, 999999999, 4, chessboard, 0, 0, 0)
    #
    # else:
    #     opencontrol=False
    # if cnt<=3:
    #     if color==-1:
    #         next=black(chessboard)
    #
    #         list.append(next)
    #     else:
    #         next=white(chessboard)
    #         list.append(next)

    if color==-1 and cnt<3 and ran==1:
        n= black(chessboard)
        if n==None:
            pass
        else:
            list.append(n)
    elif color==1 and cnt<=3 and ran==1:
        n=white(chessboard)
        if n==None:
            pass
        else:
            list.append(n)

    elif cnt>=70:
        max_(2, color, -999999999, 999999999, 2, chessboard, 0, 0, 0)
    else:
        if cnt<=8:
            max_(6, color, -999999999, 999999999, 6, chessboard, 0, 0, 0)
        # kill = vcx(color, 10, chessboard)
        # print(time.time()-sta)
        # print(kill)
        # if kill!=False:
        #     print("kill")
        #     list.append([kill[0],kill[1]])
        # else:
        else:
            max_(4, color, -999999999, 999999999, 4, chessboard, -1, 0, 0)

            if color==-1:
                kill=vcx(color,8,chessboard)

                if kill!=False:
                    list.append(kill)

            # else:
            #     print(time.time() - sta)
            #     max_(4, color, -999999999, 999999999, 4, chessboard, -1, 0, 0)

    # kill=vcx(color,8,chessboard)
    #     # print(time.time()-sta)
    #     # print(kill)
    #     # if kill!=False:
    #     #     print("kill")
    #     #     list.append([kill[0],kill[1]])
    #     # print("killfilled")


    # if cnt>=80:
    #     max_(4, color, -999999999, 999999999, 4, chessboard, 0, 0, 0)
    #
    # elif cnt>=15:
    #     kill = vcx(color, 8, chessboard)
    #     print(time.time()-sta)
    #         # print(kill)
    #     if kill!=False:
    #             # print("kill")
    #         list.append([kill[0],kill[1]])
    #     else:
    #         max_(4, color, -999999999, 999999999, 4, chessboard, -1, 0, 0)
    #
    # else:
    #     max_(6, color, -999999999, 999999999, 6, chessboard, 0, 0, 0)

    # if cnt<=6:
    #    max_(6, color, -999999999, 999999999, 6, chessboard, 0, 0, 0)
    # elif cnt>6 and cnt <10:
    #
    #     max_(4, color, -999999999, 999999999, 4, chessboard, -1, 0, 0)
    #     kill = vcx(color, 8, chessboard)
    #     print(time.time()-sta)
    #     # print(kill)
    #     if kill!=False:
    #         # print("kill")
    #         list.append([kill[0],kill[1]])
    #
    #
    #
    # elif cnt>=10:
    #     kill = vcx(color, 8, chessboard)
    #     max_(4, color, -999999999, 999999999, 4, chessboard, -1, 0, 0)
    #     print(time.time() - sta)
    #     # print(kill)
    #     if kill != False:
    #         # print("kill")
    #         list.append([kill[0], kill[1]])



    #
    #     max_(4, color, -999999999, 999999999, 4, chessboard,0)
        # negmax(chessboard,color,4,4,-9999999,9999999,0)
    # if ccc==6:
    #     list.append([14,14])
    # # global ccc
    # ccc+=1
    # if (chessboard==c3).all():
    #     list.append([8,7])
    #
    # if (chessboard==c2).all():
    #     list.append([7,9])
    #
    # if (chessboard==c4).all():
    #     list.append([9,5])
    # if (chessboard==c5).all():
    #     list.append([7,9])
    if p==1:
        if steppath[steppath.__len__()-1]==[10,8]:
            list.append([9,9])

            p=0

    if cnt<=8 and ran==1 and not testcontrol:
        pass
        # if (chessboard==c6).all():
        #     list.append([7,8])
        #
        # if (chessboard==c7).all():
        #     list.append([7,8])
        if (chessboard==c8).all():
            list.append([8,7])
        # elif (chessboard==c9).all():
        #     list.append([8,6])
        # elif (chessboard==c10).all():
        #     list.append([7,6])
        # elif (chessboard == c11).all():
        #     list.append([6,5])
        elif(chessboard==c02).all():
            list.append([5,5])


        elif(chessboard==c01).all():
            list.append([6,7])
        elif (chessboard==c12).all():
            list.append([9,5])
        elif (chessboard==c13).all():
            list.append([9,7])
        elif (chessboard==c14).all():
            list.append([5,8])
        # elif (chessboard==c15).all():
        #     list.append([6,7])
        # elif (chessboard==c16).all():
        #     list.append([6,6])
        elif (chessboard==c17).all():
            list.append([6,8])
        elif (chessboard==c20).all():
            list.append([6,6])
        elif (chessboard==c19).all():
            list.append([8,7])
        elif(chessboard==c21).all():
            list.append([6,5])
        elif(chessboard==c22).all():
            list.append([6,8])
        elif(chessboard==c23).all():
            list.append([5,8])
        elif(chessboard==c24).all():
            list.append([6,6])
        elif(chessboard==c25).all():
            list.append([7,6])
        elif(chessboard==c6).all():
            list.append([9,7])
        elif(chessboard==c00).all():
            list.append([9,9])
    elif cnt<10 and ran==0:

        if (chessboard==c18).all():
            list.append([6,7])

    print(list)


    if list.__len__()==0:
        list.append([7,7])

    chessb[list[list.__len__()-1][0],list[list.__len__()-1][1]]=color
    move(chessboard,list[list.__len__()-1][0],list[list.__len__()-1][1],color)
    global comcnt
    comcnt+=1
    cnt+=1



    # max_(chessboard,4,4,mins,maxs,color,color)
    # max_(chessboard, 4, 4, mins, maxs, color, color)

    print("leaf %d" %(leaf))
    print("abcut %d"%(abcut) )
    print("starcut %d"%(starcut))
    print("cachecut %d" %(cachecut))
    print("cnt %d"%(cnt))
    print("errorcount%d"%(errcount) )
    print("movecnt%d"%(movecnt))
    print("backcnt%d"%(backcnt))
    print("killstar%d"%(killstarcnt))
    print(time.time()-sta)

    testtt=True

    # print("step %d" %(stepp))
    # print("cache get %d" %(cacheget))


def play_play():
    chessboard=np.zeros(shape=(15,15))
    scoreinit(chessboard)
    print(blackTotal)
    print(whiteTotal)
    print(blackScore)
    print(whiteScore)

    while True:
        x=int(input("blackx"))
        y=int(input("blacky"))
        col=int(input("color"))

        move(chessboard,x,y,col)

        print(blackTotal)
        print(whiteTotal)
        print(blackScore)
        print(whiteScore)

def test8():
    chessboard = np.zeros(shape=(15,15))
    chessboard[2, 2] = 1
    chessboard[2, 4] = 1
    chessboard[3, 2:4] = 1
    chessboard[5, 2] = 1
    chessboard[1, 10:12] = -1
    chessboard[2, 10] = -1
    chessboard[4, 12:14] = -1
    print(chessboard)
    aigo(chessboard,-1)

    a = list[list.__len__() - 1][0]
    b = list[list.__len__() - 1][1]

    print([a, b])






def starCheck(p1,p2):
    if p1 == None or p2 == None:
        return False

    if abs(p1[0] - p2[0]) > 4 or abs(p1[1] - p2[1]) > 4:

        return False


    if p1[0] == p2[0] or p1[1] == p2[1] or abs(p1[0] - p2[0]) == abs(p1[1] - p2[1]):
        return True


    return False



def findmax(chessboard,color,control):
    comfive = []
    humfive = []
    comfour = []
    humfour = []
    comblockedfour = []
    humblockedfour = []
    comthree = []
    humthree = []
    for i in range(15):
        for j in range(15):

            if chessboard[i][j] != 0:
                continue


            if hasnei(chessboard,i, j, 2, 2)==False:
                continue



            scorecom = calculate4(i,j,chessboard,color,0)
            # if color==-1:
            #     scorecom=blackScore[i,j]
            #     scorehum=whiteScore[i,j]
            # else:
            #     scorecom=whiteScore[i,j]
            #     scorehum=blackScore[i,j]
            scorehum = calculate4(i,j,chessboard,-color,0)
            if attackpath.__len__() >= 1:

                if starCheck(attackpath[attackpath.__len__() - 1], [i, j])==False:
                    if control == 2 and starCheck(defenceAndAttack[defenceAndAttack.__len__() - 1], [i, j]):
                        global defenceattackCount
                        defenceattackCount+=1
                    elif scorehum >= blockfour or scorecom >= four:
                        pass
                    else:
                        #  killstarcnt
                        # killstarcnt+=1global
                        continue




            if scorecom >= five:
                comfive.append([i, j, scorecom, scorehum])

            elif scorehum >= five:
                humfive.append([i, j, scorecom, scorehum])


            elif scorecom >= four:
                comfour.append([i, j, scorecom, scorehum])


            elif scorecom >= blockfour:
                comblockedfour.append([i, j, scorecom, scorehum])


            elif scorecom >= three:
                comthree.append([i, j, scorecom, scorehum])




    if comfive.__len__() >= 1:
        return comfive

    if humfive.__len__() >= 1:
        return humfive

    if comfour.__len__() >= 1:
        return comfour



    if comblockedfour.__len__() >= 1 or comthree.__len__() >= 1:
        return comblockedfour+comthree

    return []


def findmin(chessboard,color,control):
    humfive=[]
    comfive=[]
    humanblockedfour=[]
    defence=[]


    for i in range(15):
        for j in range(15):


            if chessboard[i][j]!=0:
                continue


            if hasnei(chessboard,i,j,2,2)==False:
                continue

            scorecom=calculate4(i,j,chessboard,color,0)

            scorehum=calculate4(i,j,chessboard,-color,0)
            # if color == -1:
            #     scorecom = blackScore[i][j]
            #     scorehum = whiteScore[i][j]
            # else:
            #     scorecom = whiteScore[i][j]
            #     scorehum = blackScore[i][j]


            if attackpath.__len__()>=1 and starCheck(attackpath[attackpath.__len__() - 1], [i, j])==False:
                if control==2 and starCheck(defenceAndAttack[defenceAndAttack.__len__()-1],[i,j]):

                    pass
                elif scorehum >= blockfour:
                    pass
                else:
                    continue


            if scorehum>=five:
                humfive.append([i,j,scorehum,scorecom])

            elif scorecom>=five:
                comfive.append([i,j,scorehum,scorecom])

            elif scorehum>=blockfour:
                humanblockedfour.append([i,j,scorehum,scorecom])

            elif scorecom>=four:
                defence.append([i,j,scorehum,scorecom])

            elif scorecom>=blockfour:
                defence.append([i,j,scorehum,scorecom])






    if  humfive.__len__()>=1:
        return humfive

    if comfive.__len__()>=1:
        return comfive

    return humanblockedfour+defence


def maxvcf(com,depth,td,c,chessboard):

    if time.time()-sta>=4.7:
        return False
    # print(depth)
    candidate = findmax(chessboard,com, c)
    if depth==td:
        if candidate.__len__()>=2:

            candidate.sort(key=lambda a: a[2], reverse=True)

            candidate=candidate[0:2]

    control = 0
    if depth == 0 or candidate.__len__() == 0:
        # print(depth)
        return False


    for i in range(candidate.__len__()):
        tempx = candidate[i][0]
        tempy = candidate[i][1]
        scom = candidate[i][2]
        shum = candidate[i][3]
        if scom >= five:
            if depth == td:
                return [tempx, tempy]

            return True

        if shum >= five:

            control = 1
            if scom >= three:
                control=2
                defenceAndAttack.append([tempx, tempy])




        chessboard[tempx][tempy] = com
        # move(chessboard,tempx,tempy,com)
        if control==0:
            attackpath.append([tempx, tempy])




        path.append([tempx, tempy, com])
        l = minvcf(-com, depth - 1, td, control,chessboard)
        chessboard[tempx][tempy] = 0
        # back(chessboard,tempx,tempy)
        path.pop()
        if control==0:
            attackpath.pop()
        if control==2:
            defenceAndAttack.pop()




        if l == False:

            if depth == td:
                return [tempx, tempy]

            return True



    return False


def minvcf(com,depth,td,c,chessboard):



    candidate = findmin(chessboard,-com, c)
    if candidate.__len__() == 0:
        return True


    for i in range(candidate.__len__()):

        tempx = candidate[i][0]
        tempy = candidate[i][1]
        s = candidate[i][2]

        if s >= five:
            return True

        chessboard[tempx][tempy] = com
        # move(chessboard,tempx,tempy,com)
        path.append([tempx, tempy, com])

        t = maxvcf(-com, depth - 1, td, c,chessboard)


        chessboard[tempx][tempy] = 0
        # back(chessboard,tempx,tempy)
        path.pop()

        if t == False:
            if depth == td - 1:
                defencepath.append([tempx, tempy])

            return True

    return False



def vcx(com,depth,chessboard):
    record=[]
    i=8
    while i<=depth:
        temp=maxvcf(com,i,i,0,chessboard)
        if temp==False:
            pass
        else:
            return temp
        i+=2


    return False






def test10():

    chessboard=np.zeros(shape=(15,15))
    chessboard[7,6:9]=-1
    chessboard[7,9]=1
    chessboard[8,7]=-1
    chessboard[8,8]=1
    chessboard[9,6:8]=1
    chessboard[10,6]=-1
    chessboard[5,6]=-1

    print(chessboard)

    aigo(chessboard,-1)
    print(list.pop())


def killtest():
    chessboard = np.zeros(shape=(15, 15))
    ttt = np.zeros(shape=(15, 15))
    # chessboard = np.zeros((self.chessboard_size, self.chessboard_size), dtype=np.int)
    chessboard[2, 2:4] = 1
    chessboard[4, 1:3] = 1
    chessboard[1, 10:12] = -1
    chessboard[2, 10] = -1
    chessboard[4, 12] = -1
    # chessboard[1,9]=-1
    # chessboard[1,8]=1
    # chessboard[3,11]=-1
    # chessboard[5,13]=1
    print(chessboard)
    scoreinit(chessboard)

    ll=vcx(-1,8,chessboard)

    print(ll)




def black(chessboard):



    if cnt==0:
        i=random.randint(0,2)
        if i==0:
            return [7,6,0]
        elif i==1:
            return [8,8,0]
            # return [7,6,0]
        elif i==2:
            return [6,7,0]

    elif cnt==1:
        a=steppath[0]
        b=steppath[1]
        if abs(a[0]-b[0])>1 or abs(a[1]-b[1])>1:
            if b==[9,5]:

                return [8,8,0]

        next=[]
        if a[0]==b[0] or a[1]==b[1]:
            heng=True
        candidate=[[a[0]-1,a[1]-1,0],[a[0]+1,a[1]+1,0],[a[0]+1,a[1]-1,0],[a[0]-1,a[1]+1,0]]
        for i in candidate:
            if chessboard[i[0],i[1]]!=0:
                continue
            sc=calculate4(i[0],i[1],chessboard,-1,0)
            if a[0]==b[0] or a[1]==b[1]:
                if sc>=two and hasnei(chessboard,i[0],i[1],1,2):
                    next.append(i)
                    # return i

            else:
                if sc>=two:
                    next.append(i)
                    # global xie
                    # xie=True

                    # return i
        r=random.randint(0,1)
        if next.__len__()==0:
            max_(4, -1, -999999999, 999999999, 4, chessboard, 0, 0, 0)
        else:
            return next[0]
    elif cnt==2:
        # i=random.randint(0,1)
        # if i==0:
        #     max_(6, -1, -999999999, 999999999, 6, chessboard, 0, 0, 0)
        #     return
        s=steppath[2]
        threel=[]
        twotwol=[]
        other =[]
        for i in range(1,14):
            for j in range(1,14):

                if chessboard[i,j]!=0:
                    continue
                if not whiteop(chessboard,i,j,1,1,-1):
                    continue

                sc=calculate4(i,j,chessboard,-1,0)

                if sc>=three:
                    threel.append([i,j])

                else:
                    if sc>=2*two:
                        if hasnei(chessboard,i,j,1,2) and abs(i-s[0]==abs(j-s[1])):
                            twotwol.append([i,j])




        if threel.__len__()>=1:
             return threel[0]

        if twotwol.__len__()>=1:

            # xie=True
            return twotwol[0]
        else:
            max_(6, -1, -999999999, 999999999, 6, chessboard, 0, 0, 0)



    elif cnt==3:
        # max_(6, -1, -999999999, 999999999, 6, chessboard, 0, 0, 0)
        l=steppath[4]
        humfour=[]
        next=[]

        for i in range(0,15):
            for j in range(0,15):
                if chessboard[i,j]!=0:
                    continue
                hum=calculate4(i,j,chessboard,1,0)
                if hum>=four:
                    humfour.append([i,j,0])
                if not whiteop(chessboard,i,j,1,1,-1):
                    continue
                # com = calculate4(i, j, chessboard, -1, 0)


                if bagua(chessboard,i,j,-1) and (l[0]==i or l[1]==j):
                    next.append([i,j,0])
        if humfour.__len__()>=1:
            return humfour
        elif next.__len__()>=1:
            return next
        max_(6, -1, -999999999, 999999999, 6, chessboard, 0, 0, 0)









def white(chessboard):

    next=steppath[steppath.__len__()-1]
    tempx=next[0]
    tempy=next[1]
    if cnt==1:
        print("zy1")
        i=random.randint(0,2)
        print(i)
        if i==0:
            return [tempx+1,tempy+1]
        elif i==1:
            return [tempx+1,tempy+1]
        elif i==2:
            return [tempx+1,tempy+1]

    elif cnt==2:
        print("zy2")

        a=[]
        b=[]

        for i in range(0,15):
            for j in range(0,15):
                if chessboard[i,j]!=0:
                    continue
                if not hasnei(chessboard,i,j,1,1):
                    continue

                hum=calculate4(i,j,chessboard,-1,0)
                com=calculate4(i,j,chessboard,1,0)

                if hum>=three and com>= two:
                    a.append([i,j])
                elif com>=two:
                    b.append([i,j])
        if a.__len__()>=1:
            return a[0]
        return b[0]

    elif cnt==3:
        print("zy3")

        fir=steppath[0]
        sec=steppath[2]
        third=steppath[4]

        if bagua(chessboard,third[0],third[1],-1) and (third[0]==sec[0] or third[1]==sec[1]):
            print("laowjakjadh")


            if chessboard[8,9]==-1:
                return [8,6]
            elif chessboard[8,5]==-1:
                return [8,8]
            elif chessboard[6,5]==-1:
                return [6,8]
            elif chessboard[6,9]==-1:
                return [7,6]
            x=third[0]-sec[0]
            y=third[1]-sec[1]
            x=sec[0]-x
            y=sec[1]-y
            return [x,y]
        else:

            max_(6, 1, -999999999, 999999999, 6, chessboard, 0, 0, 0)































class AI(object):

    def __init__(self,chessboard_size,color,time_out):
        self.chessboard_size=chessboard_size
        self.color=color
        self.time_out=time_out

        self.candidate_list=[]
        # global ifinit

        # ifinit=False



    def go(self,chessboard):
        global ifinit,chessb

        # step=0
        self.candidate_list.clear()
        global aaa
        # print(aaa)
        # aaa+=1



        # start=time.time()
        aigo(chessboard,self.color)
        # if list.__len__()==0:
        #     self.candidate_list.append([7,7])
        # else:





        a = list[list.__len__() - 1][0]
        b = list[list.__len__() - 1][1]
        # print([a,b])
        # chessb=chessboard
        # chessb[a,b]=self.color



            # a=14
            # b=14

        self.candidate_list.append([a,b])


        # print(countt)
        # print(step)
            # print(list)

        # if(list.__len__()!=0):
        #
        #     # y=a[0][1]
        #
        #     a = list[list.__len__() - 1][0]
        #     b = list[list.__len__() - 1][1]
        #
        #     self.candidate_list.append([a,b])
        # else:
        #     self.candidate_list.append([14,14])
        list.clear()




def testkill():
    chessboard=np.zeros(shape=(15,15))
    chessboard[7,5:8]=-1
    chessboard[8,6]=-1
    chessboard[9,7]=-1
    chessboard[6,5]=-1
    chessboard[6,6:9]=1
    chessboard[6,4]=1
    chessboard[7,4]=1
    chessboard[9,5]=1

    kill=vcx(-1,12,chessboard)
    print(chessboard)
    print(kill)



if __name__ == '__main__':
    # test1()
    # testkill()
    # test5()
    print(c12)
    play()

    # # killtest()
    # test1()
    # # test()
    # print(c1)
    # print
    # play_play()

    chessboard=np.zeros(shape=(15,15))
    scoreinit(chessboard)
    move(chessboard,3,7,1)
    move(chessboard,5,7,1)
    move(chessboard,7,7,1)

    # calculate(3,7,chessboard,0,1,0)
    move(chessboard,9,7,1)
    move(chessboard,11,7,1)
    # for i in range(15):
    #     for j in range(15):
    #         whiteScore[i,j]=calculate(i,j,chessboard,0,1,0)
    print(whiteScore)

    print()

    # play()
    # chessboard=np.zeros(shape=(15,15),dtype=np.int)
    # zabristinit()
    # print(init)
    # move(chessboard,7,7,-1)
    # print(init)
    # back(chessboard,7,7)
    # print(init)
    # #
    # move(chessboard,7,7,-1)
    #
    # move(8,8,1)

    # chessboard=np.zeros(shape=(15,15))
    # move(chessboard,7,7,-1)
    # move(chessboard,6,7,-1)
    # # move(chessboard,4,7,-1)
    # move(chessboard,9,7,-1)
    # print(chessboard)
    #
    # print(blackScore)
    # # print(whiteScore)
    # print(blackTotal)
    # print(whiteTotal)
    # test8()
    # play_play()
    #  test1()
    # test7()

    # test5()

    # ppp=(1,2)
    # ppp[0]=2
    #
    # print(ppp)
    # chessboard=np.zeros(shape=(15,15))
    # chessboard[7,7]=-1

   # test11()
    # test1()
    # play()
   #  test6()
    # test3()
    # zabristinit()
    #
    # print(init)
    # zabmove(7,7,-1)
    #
    # print(init)
    #
    # zabmove(7,7,-1)
    #
    # print(init)
    # chessboard = np.zeros(shape=(15, 15))

    # zabristinit()
    # move(chessboard,2,2,1)
    # print(init)
    # back(chessboard,2,2)
    # print(init)



    # print(init)
    # move(chessboard,7,7,-1)
    # print(init)
    # # back(chessboard,7,7)
    # # print(init)
    # move(chessboard,6,6,1)
    # print(init)
    # # back(chessboard,6,6)
    # # print(init)
    # move(chessboard,6,7,-1)
    # print(init)
    # move(chessboard,7,6,1)
    # print(init)
    # print()
    # back(chessboard,7,6)
    # print(init)
    # back(chessboard,6,7)
    # back(chessboard,6,6,)
    # back(chessboard,7,7)
    # print(init)
    #
    # move(chessboard,7,6,1)
    # move(chessboard,6,7,-1)
    # move(chessboard,6,6,1)
    # move(chessboard,7,7,-1)
    # print(init)








    # a=[]
    # b=[1,2]
    # print(a+b)

    # a=[[1,2,9],[23,43,2]]
    # b=[]
    # def t(a :[],b:[]):
    #     if a[2]>b[2]:
    #         return 1
    #     else:
    #         return 0
    #
    # a.sort(key=lambda a:a[2],reverse=True)
    # print(a)





