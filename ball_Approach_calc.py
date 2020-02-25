# Module is designed by Gleb Korostinskij

import math


def ball_Approach_Calc(ball_X_m, ball_Y_m, player_X_m, player_Y_m, dist_Between_Goals_m, dist_Between_Sides_m ):

    def rad(a):
        return a*2*3.1415927/360
    def gradus(a):
        return round(a*360/(2*3.1415927),2)

    def ugolx(x,y):
        try:
            tan=y/x
            a=math.atan(tan)
            a=round(gradus(a),2)
            return a
        except:
            if y>=0:
                return 90
            else:
                return -90

    def ProTochkaEnd(xm,ym,L):
        try:
            a = math.atan(ym/(Lx-xm))
            xp, yp = round(xm - L*math.cos(a),4),round(ym + L*math.sin(a),4)

            return round(xp,4),round(yp,4)
        except:
            return 0,0

    def ProTochkaPodhod(xm,ym,xu,yu,L,Lp):
        if xu>=xm:
            xp,yp = ProTochkaEnd(xm,ym,L)
            xp1=xp
            if yu > ym:
                yp1 = yp + Lp
            else:
                yp1 = yp - Lp
            return round(xp1,4),round(yp1,4)
        else:
            return None

    def Napravlenie(x1,y1,x2,y2):
        nx=x2-x1
        ny=y2-y1
        r=math.sqrt(nx**2+ny**2)
        try:
            nx/=r
            ny/=r
        except:
            nx=nx
        return round(nx,4),round(ny,4)


    def ugol(xn0,yn0,xn1,yn1):
        try:
            xn1 /= math.sqrt((xn1**2 + yn1**2))
            yn1 /= math.sqrt((xn1**2 + yn1**2))
            xn0 /= math.sqrt((xn0**2 + yn0**2))
            yn0 /= math.sqrt((xn0**2 + yn0**2))
            a1 = math.acos((xn0*xn1 + yn0*yn1) )
            a1 = round(gradus(a1),2) 

            if yn1>=yn0:
                return a1
            else:
                return -a1
        except:
            return 0

    #print(ugol(1,0,-1,0))

    #Lx =  1.8  #поле по x 3 метра
    #Ly = 1.3     #поле по y 4 метра
    Lx = dist_Between_Goals_m / 2
    Ly = dist_Between_Sides_m / 2
    xm, ym = ball_X_m, ball_Y_m  #координаты мяча
    xu, yu = player_X_m, player_Y_m  #координаты игрока
    L = 0.15  #расстояние от точки прихода до мяча
    Lp = 0.3 #расстояние от точки 2 дло точки 1 по оси y
    a0 = 0   #начальный угол
    print('Начальная точка: ',xu,yu)
    print('Начальное направление: ',a0)

    a0 = rad(a0)

    xn0, yn0 = Napravlenie(0,0, 1, math.sin(a0))  #начальный вектор направления

    xpk, ypk = ProTochkaEnd(xm,ym,L)  #Вычисление Конечной Промежуточной точки

    destination = []
    stop_point = []

    if ProTochkaPodhod(xm,ym,xu,yu,L,Lp)!=None:

        xpb,ypb=ProTochkaPodhod(xm,ym,xu,yu,L,Lp)
        print('Промежуточная точка: ', xpb,ypb)
        xn1,yn1=Napravlenie(xu,yu,xpb,ypb)
        a1=ugol(xn0,yn0,xn1,yn1)
        xn2,yn2=Napravlenie(xpb,ypb,xpk,ypk)
        a2=ugol(xn1,yn1,xn2,yn2)

        xn3,yn3=Napravlenie(Lx,0,xm,ym)


        ax1=ugolx(xn1,yn1)
        ax2=ugolx(xn2,yn2)
        ax3=ugolx(xn3,yn3)
        print("Конечная точка: ",xpk,ypk)

        #print('a1 = ',a1)
        print('ax1 = ',ax1)
       # print('a2 = ',a2)
        print('ax2 = ',ax2)
        print('ax3 = ',ax3)

        propodhod=True
        stop_point = xpb, ypb, ax2
        destination.append(stop_point)
        stop_point = xpk,ypk, ax3
        destination.append(stop_point)
    else:
        xn1,yn1=xn0,yn0
        xn2,yn2=Napravlenie(xu,yu,xpk,ypk)
        a1=ugol(xn1,yn1,xn2,yn2)

        xn3,yn3=Napravlenie(Lx,0,xm,ym)
        a2=ugol(xn2,yn2,xn3,yn3)


        ax1=ugolx(xn2,yn2)
        ax2=ugolx(xn3,yn3)
    
        print("Конечная точка: ",xpk,ypk)
     #   print('a1 = ',a1)
        print('ax1 = ',ax1)
     #   print('a2 = ',a2)
        print('ax2 = ',ax2)
        propodhod=False

        stop_point = xpk,ypk, ax2
        destination.append(stop_point)
    return destination

















