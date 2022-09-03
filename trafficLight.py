#シグナルの出し方
#各ステップ、車が通ったら長さ10のキューに追加、このキューは送信側の信号機が持つ
#[0, 0, 0, 1, 1, 1, 0, 0, 0, 0]
#上のキューのsumが2を超えたらシグナルを送る
#[0, 0, 1, 1, 1, 1, 0, 0, 0, 0]
#元の協調方法を採用している、これが完成形


import random
import copy
import numpy as np
import fielddata
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
from collections import deque

class Allcar:
    def __init__(self, num):
        self.__pos = [] #座標、進行方向
        self.__stoppos = [] #停止座標、進行方向情報は含まない
        self.__countpos = np.array([0] * (num * 2))# 各停止場所で止まっている車の数
        self.__throughcountpos = np.array([0] * (num * 4))#交差点を通過した数、信号機×4個
        self.__endposhis, self.__endstopposhis = [], [] #[[0ステップ目の位置一覧],[1ステップ目の位置一覧],...]
        self.__allpos, self.__allspos = [], []
        self.congestionnum = [[0, 0] for i in range(num)] #次の信号に向かう車の数
        self.carqueue = [[deque([0]) * 10, deque([0]) * 10] for _ in range(num)] #直近の通過した車を保存
        self.congestionnum = [[0, 0] for _ in range(num)] #各ステップで車が通過した数の累計

    #移動、lestposをdeciposに入れる間に未移動車、停止車、黄色停止車も振り分ける
    #self.posではndarray使わない
    #フラグについて、0:なし, 1:右折準備,左折なし, 2:右折完了,左折なし, 3:右折なし,左折完了, 4:右折準備,左折完了, 5:右折完了,左折完了
    def move(self, light, time):
        # if step == 300:
        #     self.situation()
        decipos = []  # 確定している車、すべて
        untreatedpos = []  # 今回のループでは決めきれない車
        self.__stoppos = []  # 停止中の車の座標、座標のみ
        yellowpos = []  # 黄色で停止しているが罰には含まれない車、座標のみ
        turnpos = []  # 右折待ちしている車、座標のみ
        # if step == 800:
        #     print(self.__pos)

        # for _ in range(200):
        #     print("")
        loop = 0
        lstLeftDown = list(
            {i[0] + 1 for i in fielddata.intersection})  # 左、下に向かう車線の座標、リバース[12, 25, 38, 51]
        lstRightUp = list({i[0] for i in fielddata.intersection})  # 右、上に向かう車線の座標[11, 24, 37, 50]
        lestpos1, lestpos2 = [[] for _ in range(len(lstRightUp) * 2)], [[] for _ in
                                                                        range(len(lstRightUp) * 2)]
        addx, addy, subx, suby = [], [], [], []  # 道路の終わり、並列に1動かす
        lestpos = []

        if self.__pos != []:  # self.posを各lestpos, add, subに振り分ける、あるいは消す
            for i in self.__pos:
                if [i[0], i[1]] in fielddata.dispos:  # 消失地点にいる場合
                    continue

                for j in range(len(lstRightUp)):
                    if i[0] == lstRightUp[j]:  # 右向きレーン上
                        if 51 < i[1] < 64:  # field内最奥の道路にいる時
                            addy.append(i)
                        else:
                            lestpos2[j].append(i)
                    elif i[0] == lstLeftDown[j]:  # 左
                        if -1 < i[1] < 11:
                            suby.append(i)
                        else:
                            lestpos1[j].append(i)
                    elif i[1] == lstRightUp[j]:  # 上
                        if -1 < i[0] < 11:
                            subx.append(i)
                        else:
                            lestpos1[j + len(lstRightUp)].append(i)
                    elif i[1] == lstLeftDown[j]:  # 下
                        if 51 < i[0] < 64:
                            addx.append(i)
                        else:
                            lestpos2[j + len(lstRightUp)].append(i)
                    else:
                        continue
                    break
            # print(addx, addy, subx, suby)

            addx, addy, subx, suby = np.array(addx), np.array(addy), np.array(subx), np.array(
                suby)  # 最奥道路にいる車の移動
            # print(addx.ndim, addy.ndim, subx.ndim, suby.ndim)
            if addx.ndim == 2: addx[:, 0] += 1
            if addy.ndim == 2: addy[:, 1] += 1
            if subx.ndim == 2: subx[:, 0] -= 1
            if suby.ndim == 2: suby[:, 1] -= 1
            addx, addy, subx, suby = addx.tolist(), addy.tolist(), subx.tolist(), suby.tolist()
            # print(addx, addy, subx, suby)
            decipos = decipos + addx + addy + subx + suby

            for i in range(len(lestpos1)):
                lestpos += sorted(lestpos1[i])
            for i in range(len(lestpos2)):
                lestpos += sorted(lestpos2[i], reverse=True)

        for i in fielddata.intersection:  # ハマりパターン1
            stlst = [[i[0], i[1], 1], [i[0], i[1] + 1, 0], [i[0] + 1, i[1], 2],
                     [i[0] + 1, i[1] + 1, 3]]  # ハマりパターン
            ndlestpos = np.array(copy.deepcopy(lestpos))[:, :3].tolist()
            if stlst[0] in ndlestpos and stlst[1] in ndlestpos and stlst[2] in ndlestpos and stlst[
                3] in ndlestpos:
                klst = [ndlestpos.index(stlst[0]), ndlestpos.index(stlst[1]),
                        ndlestpos.index(stlst[2]), ndlestpos.index(stlst[3])]  # lestposの番号
                klst = [lestpos[klst[0]][3], lestpos[klst[1]][3], lestpos[klst[2]][3],
                        lestpos[klst[3]][3]]
                # print(klst)
                temlst = copy.deepcopy(klst)
                for j in range(len(klst)):
                    if temlst[j] == 1 or temlst[j] == 4:
                        temlst[j] += 1
                alst = [temlst[2], temlst[0], temlst[3], temlst[1]]
                adlst = [[i[0], i[1], 2], [i[0], i[1] + 1, 1], [i[0] + 1, i[1], 3],
                         [i[0] + 1, i[1] + 1, 0]]
                for j in range(len(stlst)):
                    decipos.append(adlst[j] + [alst[j]])
                    lestpos.remove(stlst[j] + [klst[j]])

        while True:
            lestposnum = 0
            for i in lestpos:  # 全車の処理ループ
                # if i[0] in [24, 25, 37, 38, 50, 51] and i[1] in [24, 25, 37, 38, 50, 51]:
                #     print("this time process is", i)
                lestposnum += 1
                x, y, j, k = i[0], i[1], i[2], i[3]

                if np.array(copy.deepcopy(lestpos[lestposnum:])).ndim == 1:
                    ndlestpos = [[]]
                else:
                    ndlestpos = np.array(copy.deepcopy(lestpos[lestposnum:]))[:,
                                :2].tolist()  # lestpos座標、今処理中の車は含まない

                if np.array(copy.deepcopy(decipos)).ndim == 1:
                    nddecipos = [[]]
                else:
                    nddecipos = np.array(copy.deepcopy(decipos))[:, :2].tolist()  # decipos座標

                if np.array(copy.deepcopy(untreatedpos)).ndim == 1:
                    ndunpos = [[]]
                else:
                    ndunpos = np.array(copy.deepcopy(untreatedpos))[:,
                              :2].tolist()  # untreatedpos座標

                # 消失確認

                # 右折待ちするかは2パターン
                # 1:移動前の車が左前にいる場合⇒ここで処理
                # 2:移動済みの車が目の前にいる場合⇒後に処理
                if k == 1 or k == 4:
                    if fielddata.bumppos[tuple([x, y])] in ndlestpos + ndunpos:
                        decipos.append(i)
                        turnpos.append(i[:2])
                        continue
                # 信号確認
                if k == 0 or k == 2 or k == 3 or k == 5:  # 右折する車は信号の影響を受けない
                    flag = False  # ここで停止処理したらTrueに
                    for l in range(len(light)):  # 全信号機の停止線で停止するか
                        if ([x, y] in fielddata.stopline[2 * l] and light[l] == "red") or \
                                ([x, y] in fielddata.stopline[2 * l + 1] and light[l] == "green"):
                            self.__stoppos.append(i[:2])  # 停止線で停止
                            decipos.append(i)
                            flag = True
                            break
                        elif ([x, y] in fielddata.stopline[2 * l] or [x, y] in fielddata.stopline[
                            2 * l + 1]) and \
                                light[l] == "yellow":
                            yellowpos.append(i[:2])
                            decipos.append(i)  # 停止するが罰に含めない
                            flag = True
                            break
                        elif ([x, y] in fielddata.stopline[2 * l] or [x, y] in fielddata.stopline[
                            2 * l + 1]) and \
                                light[l] == "black":
                            raise ValueError("car is going to go into black trafficlight")
                    if flag: continue

                ####
                if time == 0:  # 通行止めポイント
                    clst = copy.deepcopy([[24, 24], [24, 37], [24, 11]])
                elif time == 1:  # [24, 37] - [37, 37]が通行止め
                    clst = copy.deepcopy([[37, 24], [37, 37], [37, 11]])
                closeflag = 0
                for cpos in clst:
                    if [x, y, j] == [cpos[0], cpos[1], 1] and (
                            k == 2 or k == 3 or k == 0):  # 右折禁止⇒直進のみ(2/3)
                        if [x, y + 1] in ndlestpos + ndunpos:
                            untreatedpos.append(i)
                            break
                        elif [x, y + 1] in turnpos or [x, y + 1] in nddecipos:
                            decipos.append(i)
                            # turnpos.append(i[:2])
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y + 1] in self.__stoppos:
                            decipos.append(i)
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y + 1] in yellowpos:
                            decipos.append(i)
                            yellowpos.append(i[:2])
                            break
                        else:
                            decipos.append([x, y + 1, 1, k])
                            break
                    elif [x, y, j] == [cpos[0] + 1, cpos[1] + 2, 3]:  # 左折準備禁止⇒直進のみ
                        if [x, y - 1] in ndlestpos + ndunpos:
                            untreatedpos.append(i)
                            break
                        elif [x, y - 1] in turnpos or [x, y - 1] in nddecipos:
                            decipos.append(i)
                            # turnpos.append(i[:2])
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y - 1] in self.__stoppos:
                            decipos.append(i)
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y - 1] in yellowpos:
                            decipos.append(i)
                            yellowpos.append(i[:2])
                            break
                        else:
                            decipos.append([x, y - 1, 3, k])
                            break
                    elif [x, y, j] == [cpos[0], cpos[1] + 1, 0] and (
                            k != 1 and k != 4):  # 直進禁止⇒右折のみ(2/3)
                        if [x + 1, y] in ndlestpos + ndunpos:
                            untreatedpos.append(i)
                            break
                        elif [x + 1, y] in turnpos or [x + 1, y] in nddecipos:
                            decipos.append(i)
                            # turnpos.append(i[:2])
                            self.__stoppos.append(i[:2])
                            break
                        elif [x + 1, y] in self.__stoppos:
                            self.__stoppos.append(i[:2])
                            decipos.append(i)
                            break
                        elif [x + 1, y] in yellowpos:
                            decipos.append(i)
                            yellowpos.append(i[:2])
                            break
                        else:
                            decipos.append([x + 1, y, 3, 5]) if k == 3 else decipos.append(
                                [x + 1, y, 3, 2])
                            break





                    elif [x, y, j] == [cpos[0] + 14, cpos[1] + 1, 3] and (
                            k == 2 or k == 0 or k == 3):  # 右折禁止⇒直進のみ
                        if [x, y - 1] in ndlestpos + ndunpos:
                            untreatedpos.append(i)
                            break
                        elif [x, y - 1] in turnpos or [x, y - 1] in nddecipos:
                            decipos.append(i)
                            # turnpos.append(i[:2])
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y - 1] in self.__stoppos:
                            decipos.append(i)
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y - 1] in yellowpos:
                            decipos.append(i)
                            yellowpos.append(i[:2])
                            break
                        else:
                            decipos.append([x, y - 1, j, k])
                            break
                    elif [x, y, j] == [cpos[0] + 13, cpos[1] - 1, 1]:  # 左折準備禁止⇒直進のみ
                        if [x, y + 1] in ndlestpos + ndunpos:
                            untreatedpos.append(i)
                            break
                        elif [x, y + 1] in turnpos or [x, y + 1] in nddecipos:
                            decipos.append(i)
                            # turnpos.append(i[:2])
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y + 1] in self.__stoppos:
                            decipos.append(i)
                            self.__stoppos.append(i[:2])
                            break
                        elif [x, y + 1] in yellowpos:
                            decipos.append(i)
                            yellowpos.append(i[:2])
                            break
                        else:
                            decipos.append([x, y + 1, 1, k])
                            break

                    elif [x, y, j] == [cpos[0] + 14, cpos[1], 2] and (
                            k != 1 and k != 4):  # 直進禁止⇒右折しかない(2/3)
                        if [x - 1, y] in ndlestpos + ndunpos:
                            untreatedpos.append(i)
                            break
                        elif [x - 1, y] in turnpos or [x - 1, y] in nddecipos:
                            decipos.append(i)
                            # turnpos.append(i[:2])
                            self.__stoppos.append(i[:2])
                            break
                        elif [x - 1, y] in self.__stoppos:
                            decipos.append(i)
                            self.__stoppos.append(i[:2])
                            break
                        elif [x - 1, y] in yellowpos:
                            decipos.append(i)
                            yellowpos.append(i[:2])
                            break
                        else:
                            decipos.append([x - 1, y, 1, 5]) if k == 3 else decipos.append(
                                [x - 1, y, 1, 2])
                            break
                    else:
                        closeflag += 1
                if closeflag != 3:
                    continue

                ####移動後を考える領域

                xlst = [x + 1, x, x - 1, x]
                ylst = [y, y + 1, y, y - 1]

                p = copy.deepcopy([xlst[j], ylst[j]])  # 次の移動先

                # 次の位置に位置決定車、停止車、未移動車、黄色停止車がいない場合⇒そのまま進める
                if (p not in (
                        ndlestpos + ndunpos) and p not in nddecipos and p not in self.__stoppos and
                        p not in yellowpos and p not in turnpos):
                    if p in fielddata.turnleftpos[j] and (k == 0 or k == 2):  # 左折位置
                        rand = random.randint(1, 3)
                        if rand == 3 and k == 0:
                            decipos.append([xlst[j], ylst[j], (j + 1) % 4, 3])  # 次左折
                        elif rand == 3 and k == 2:
                            decipos.append([xlst[j], ylst[j], (j + 1) % 4, 5])  # 次左折
                        else:
                            decipos.append([xlst[j], ylst[j], j, k])
                    elif p in fielddata.turnrightpos[j] and (k == 0 or k == 3):  # 右折位置
                        rand = random.randint(1, 2)
                        if rand == 2 and k == 0:
                            decipos.append([xlst[j], ylst[j], (j - 1) % 4, 1])  # 次右折
                        elif rand == 2 and k == 3:
                            decipos.append([xlst[j], ylst[j], (j - 1) % 4, 4])
                        else:
                            decipos.append([xlst[j], ylst[j], j, k])
                    else:
                        if k == 1:
                            decipos.append([xlst[j], ylst[j], j, 2])
                        elif k == 4:
                            decipos.append([xlst[j], ylst[j], j, 5])
                        else:
                            decipos.append([xlst[j], ylst[j], j, k])
                    # decipos.append([xlst[j], ylst[j], j, k])
                # 次の位置に停止車がいる場合⇒自車も停止
                elif p in self.__stoppos:
                    self.__stoppos.append(i[:2])  # 停止線で停止
                    decipos.append(i)
                # 次の位置に黄色停止車がいる場合
                elif p in yellowpos:
                    yellowpos.append(i[:2])
                    decipos.append(i)
                # 次の位置に右折停止車がいる場合
                elif p in turnpos:
                    turnpos.append(i[:2])
                    decipos.append(i)
                # 次の位置に未移動車がいる場合：一旦自車の処理を飛ばす
                elif p in ndlestpos + ndunpos:
                    untreatedpos.append(i)
                # それ以外(右直事故パターン、目の前にdeciposがいる)：停止
                else:
                    turnpos.append(i[:2])
                    decipos.append(i)
            loop += 1

            if len(untreatedpos) == 0:  # 全車移動後
                self.stopcount(self.__stoppos, len(light))  # 停止車の数え上げ
                decipos = self.throughcount(copy.deepcopy(decipos), len(light))
                self.__pos = copy.deepcopy(decipos)  # 決定した次の車の位置を車の位置に変更
                # self.addgenerate(time)
                # print("length", len(self.__pos))
                # print("s", self.__stoppos)
                # print("y", yellowpos)
                # print("t", turnpos)
                # x = copy.deepcopy(self.__pos)
                # x.sort()
                # print("p", x)
                break

            if loop > 100:  # 交差点が完全に詰まる場合：車を消去(改善の余地)
                print("delete")
                # self.situation()
                # self.subcar()
                self.move(light, time)
                return

            lestpos = copy.deepcopy(untreatedpos)
            untreatedpos = []
            # print("reloaded")

    def stopcount(self, stoppos, num): #停止車を場所ごとに数え上げ
        for i in stoppos: #全停車中の車の処理
            p = [i[0], i[1]]
            for j in range(num*2): #numは信号の数
                if p in fielddata.nearpos[j]:
                    self.__countpos[j] += 1

    def throughcount(self, decipos, num): #場所ごとの停止車数え上げ+シグナルの準備
        for i in range(len(decipos)):
            for j in range(num * 4):
                if decipos[i][:2] == fielddata.throughpos[j]:
                    self.__throughcountpos[j] += 1
                    self.congestioncount(j, num)
        return decipos

    def congestioncount(self, j, num): #車が通過したかどうかを記録する
        if j in fielddata.congestionsignal.keys():
            pos = fielddata.congestionsignal[j]
            self.congestionnum[pos[0]][pos[1]] += 1


    def checksignal(self): #直近n回の車の通過状態を確認、混雑しているならシグナルを送る
        lst =[[0, 0] for _ in range(len(self.congestionnum))]
        for i in range(len(self.congestionnum)): #信号機番号
            for j in range(len(self.congestionnum[i])): #縦、横
                self.carqueue[i][j].popleft()
                self.carqueue[i][j].append(self.congestionnum[i][j])
                if sum(self.carqueue[i][j]) > 1:
                    lst[i][j] = 1
                    # print(i, j, "signal!")
                    # car.situation()
        # print("fin")
        self.congestionnum = [[0, 0] for _ in range(len(self.congestionnum))] #初期化
        return lst

    def situation(self): #各ステップのfieldを表示、デバック用
        xlst, ylst, xinter, yinter = [], [], [], []
        # for j in [self.__allpos[-4], self.__allpos[-3], self.__allpos[-2], self.__allpos[-1], self.__pos]:
        for i in copy.deepcopy(self.__pos):
            xlst.append(i[1])
            ylst.append(-i[0])
        plt.scatter(xlst, ylst, s = 10)
        for i in fielddata.intersectionpos:
            xinter.append(i[1])
            yinter.append(-i[0])
        plt.scatter(xinter, yinter, c = "yellow", s = 20)
        plt.show()


    def bumpcheck(self, step, lightlst, time): #処理が遅いならlightlst削除
        for i in range(len(self.__pos)):
            for j in range(i + 1, len(self.__pos)):
                if self.__pos[i][:2] == self.__pos[j][:2]:
                    print(self.__pos[i], self.__pos[j])
                    print(lightlst)
                    self.situation()
                    raise ValueError("car collided!")
            # x = [self.__pos[i][0], self.__pos[i][1]]
            # if time == 0 and (x == [36, 24] or x == [36, 25] or x == [26, 24] or x == [26, 25]):
            #     raise ValueError("stop")
            # elif time == 1 and (x == [36, 37] or x == [36, 38] or x == [26, 37] or x == [26, 38]):
            #     raise ValueError("stop")

    def sreset(self, i): #停車・通過車数を返し、初期化
        x, y = self.__countpos[2 * i], self.__countpos[2 * i + 1]
        down, right, up, left = self.__throughcountpos[4*i], self.__throughcountpos[4*i+1], \
                                self.__throughcountpos[4*i+2], self.__throughcountpos[4*i+3]
        self.__countpos[2 * i], self.__countpos[2 * i + 1] = 0, 0  # 初期化
        self.__throughcountpos[4*i:4*(i+1)] = [0, 0, 0, 0] #初期化
        return x, y, down + right + up + left

    def reset(self, num): #初期化
        self.__pos = []
        self.__stoppos = []
        self.__countpos = np.array([0] * (num * 2))
        self.__throughcountpos = np.array([0] * (num * 4))
        self.__endposhis, self.__endstopposhis = [], []
        self.congestionnum = [[0, 0] for i in range(num)] #車の通過状況の初期化

    def generate(self, gennum, time): #生成
        t = 0
        x = random.random()
        # 生成場所があるか確認
        # if time == 0:
        #     genpos = copy.deepcopy(fielddata.genspe1)
        # else:
        #     genpos = copy.deepcopy(fielddata.genspe2)
        genpos = fielddata.genspe1 + fielddata.genspe2
        for j in range(len(self.__pos)):
            if self.__pos[j] in genpos:
                genpos.remove(self.__pos[j])  # 生成できないので削除する
            if len(genpos) < gennum:
                print("Insufficient number of car generations")
                gennum -= 1
        if len(genpos) != 0: #車の生成場所がある場合
            while True:
                #車の生成と追加
                newpos = random.choice(genpos) #新しく追加する車の座標
                self.__pos.append(newpos)
                genpos.remove(newpos)
                t += 1
                if t == gennum: return
        else: #車の生成場所がない⇒生成しない
            print("Can't generate Car")
            return

    # def addgenerate(self, time):
    #     lst = copy.deepcopy([[12, 36], [25, 36], [38, 36], [51, 36]])
    #     lst2 = copy.deepcopy([[49, 11], [49, 24], [49, 37], [49, 50]])
    #     x = np.array(self.__pos)[:, :2].tolist()
    #     rand = random.randint(0, 2)
    #     if time == 1 and rand == 0:
    #         for i in lst:
    #             if i not in x:
    #                 self.__pos.append(i + [3, 5])
    #     elif time == 2 and rand == 0:
    #         for i in lst2:
    #             if i not in x:
    #                 self.__pos.append(i + [2, 5])


    def recordhistory(self): #各ステップの位置の履歴
        self.__allpos.append(self.__pos)
        self.__allspos.append(self.__stoppos)

    @property
    def allpos(self):
        return self.__allpos

    @property
    def allspos(self):
        return self.__allspos

    def farposCooperation(self): #一つ先の信号に止まっている車の量を数えて返す
        farposcount = np.array([0, 0, 0, 0])
        for i in self.__pos:
            for j in range(len(fielddata.farpos)):
                if i[:2] in fielddata.farpos[j]:
                    farposcount[j] += 1
        return farposcount // 3

class Tlight:  # 信号機1台(エージェント)、車の数(s)を見て赤か青かを決める(a)、
    act = [] #協調信号機のstate作成に使用
    def __init__(self, actlst):
        self.step = 0  # ステップ数
        self.color = "green"  # 信号の色(縦)
        self.cflag = 0
        self.alpha, self.gamma, self.epsilon = 0.3, 0.99, 0.3 #Q学習パラメータ
        self.actlst = actlst
        self.actlen = len(self.actlst) #actの総数
        self.act = 0 #初回のみ
        self.bluelen = self.actlst[0][0]  # 青の比
        self.redlen = self.actlst[0][1]  # 周期,出来れば10の倍数
        self.rewardhistory = []
        self.state = (0, 0, 0) #縦シグナル、横シグナル、time
        self.__qtable = {self.state:[0] * self.actlen}
        self.chooseact = [0] * self.actlen

    def secondinit(self):
        self.step = 0  # ステップ数
        self.color = "green"  # 信号の色(縦)
        self.cflag = 0
        self.alpha, self.gamma, self.epsilon = 0.3, 0.99, 0.1 #Q学習パラメータ
        self.actlen = len(self.actlst) #actの総数
        self.act = 0 #初回のみ
        self.bluelen = self.actlst[0][0]  # 青の時間
        self.redlen = self.actlst[0][1]  # 赤の時間
        self.rewardhistory = []
        self.state = (0, 0, 0)

    def changecolor(self): # 縦の信号の色
        self.step += 1
        if self.step == self.bluelen:
            self.color = "yellow"
        elif self.step == self.bluelen + 2:
            self.color = "red"
        elif self.step == self.bluelen + self.redlen + 2:
            self.color = "yellow"
        elif self.step == self.bluelen + self.redlen + 4:
            self.step = 0
            self.cflag = True
            self.color = "green"


    def action(self, learningflag, signal, i, actflag): #先の信号の停止車の量を導入
        rand = random.random()
        if rand > self.epsilon and learningflag:
            self.act = np.argmax(self.__qtable[self.state])
        else: #random
            self.act = random.randint(0, self.actlen - 1)
        self.bluelen = self.actlst[self.act][0]
        self.redlen = self.actlst[self.act][1]
        if actflag:
            print(i, self.act)

    @property
    def qtable(self):
        return self.__qtable


    def learn(self, reward, penalty, learningtime, signal, time): #報酬の決定とQテーブルの更新、rewardはスカラー
        # 報酬
        self.nextstate = tuple(signal) + (time,)
        if self.nextstate not in self.__qtable.keys(): #stateが初めての場合
            self.__qtable[self.nextstate] = [0] * self.actlen
        q = self.__qtable[self.state][self.act]
        max_q = max(self.__qtable[self.nextstate]) #次の状態の行動価値の最大値
        # Q(s, a) = Q(s, a) + alpha*(r+gamma*maxQ(s')-Q(s, a))
        self.__qtable[self.state][self.act] = q + (
                    self.alpha * (reward + (self.gamma * max_q) - q))
        self.state = copy.deepcopy(self.nextstate)
        self.rewardhistory.append(-np.sum(penalty))

    def reset(self):
        self.cflag = 0 #actの切り替えタイミング
        self.step = 0  # ステップ数
        self.color = "green"  # 信号の色(縦)
        self.cflag = 0

def getanimation(all_car_lst, all_stopcar_lst, all_light_lst, ran, displaytime, cooperateflag, state):
    #x⇒-y、y⇒x
    fig_scatter = plt.figure()
    plt_scatter = []
    xRange, yRange = [0, ran], [-ran, 0]
    pos = copy.deepcopy(all_car_lst)#[[[車のpos]*全台]*全ステップ]のリスト
    stoppos = copy.deepcopy(all_stopcar_lst)
    ispos = []
    for i in range(len(fielddata.intersectionpos)):
        ispos.append([fielddata.intersectionpos[i][1], -fielddata.intersectionpos[i][0]])

    for i in range(len(all_car_lst)): #各ステップ毎にプロット
        xlst, ylst, xstoplst, ystoplst, interlst = [], [], [], [], []
        for j in range(len(all_light_lst[i])):#信号機の点
            if all_light_lst[i][j] == "green":
                intersection = plt.scatter(ispos[j][0], ispos[j][1], c="green", s = 100)
            elif all_light_lst[i][j] == "red":
                intersection = plt.scatter(ispos[j][0], ispos[j][1], c="red", s = 100)
            elif all_light_lst[i][j] == "yellow":
                intersection = plt.scatter(ispos[j][0], ispos[j][1], c="yellow", s = 100)
            else:
                intersection = plt.scatter(ispos[j][0], ispos[j][1], c="black", s = 100)
            interlst.append(intersection)

        for j in pos[i]:
            if j[:2] not in stoppos[i]:
                xlst.append(j[1])
                ylst.append(-j[0])
            else:
                xstoplst.append(j[1])
                ystoplst.append(-j[0])
        x_scatter = plt.scatter(xlst, ylst, c="blue", s = 15)  # 車の点
        x_stopscatter = plt.scatter(xstoplst, ystoplst, c="red", s = 15) #停止車の点
        title = plt.text((xRange[0] + xRange[1]) / 2, yRange[1],
                        'time={:d} (signals indicate vertical traffic)'.format((i // 50) % 20),
                         ha='center', va='bottom', fontsize='large')

        #散布図の作成
        plt_scatter.append([x_scatter, x_stopscatter, title] + [interlst[i] for i in range(len(all_light_lst[0]))])

    plt.xlim(xRange[0],xRange[1])
    plt.ylim(yRange[0],yRange[1])
    plt.grid(True)

    ani = animation.ArtistAnimation(fig_scatter, plt_scatter, interval=displaytime) #描画
    # 保存
    lst =[["NO_cooperation_start.gif", "NO_cooperation_end.gif"], ["cooperation_start.gif", "cooperation_end.gif"]]
    ani.save(lst[cooperateflag][state], writer=PillowWriter())
    #表示
    plt.show()

def getgraph(rewardlst, rewardstep, namelst): #グラフの作成
    for i in range(len(rewardlst)):
        x = rewardstep[i]
        y = rewardlst[i]
        plt.plot(x, y, label = namelst[i])
    plt.legend()
    plt.show()

def learning(gennum, lightnum, cooperationflag, learningflag, timelength, timescale, learningtime, actlst,\
             car, Tlst, lighthistory, alllight, actflag = 0):
    lastpenalty = [0] * lightnum
    lastlearningflag = 0 #学習の最後を示す
    lighthistory = []
    # congestionflag = False

    def rnd(num):
        return round(num, 1)

    for step in range(learningtime + 1): #各ステップ
        if step % 200 == 0:#真ん中の信号機に向かう車の数調整
            if step % 400 == 0:
                time = 0  # 0ランダム、1横直進、2縦直進
            else:
                time = 1
            print("generate_type", time, ", learningtime", step)
        if step % 2 == 0:
            car.generate(gennum, time)
        lightlst = [i.color for i in Tlst]  # 信号の色リスト
        if cooperationflag:
            signal = car.checksignal() #シグナルを出すかどうか
        else:
            signal = [[0, 0] for i in range(lightnum)]
        car.move(lightlst, time)  # 車の移動、停止車のカウント、戻り値は信号待ち数
        for i in range(len(Tlst)):  # 信号の変化、報酬の確認、学習、次の行動
            Tlst[i].changecolor()
            if Tlst[i].cflag: #信号が1周期終わる時
                vertical, horizontal, through = car.sreset(i)
                reward = - vertical - horizontal + through
                penalty = np.array([vertical, horizontal])
                Tlst[i].learn(reward, penalty, step, signal[i], time) # 学習
                Tlst[i].action(learningflag, signal[i], i, actflag)  # 行動の決定
                Tlst[i].reset()
        car.bumpcheck(step, lightlst, time) #衝突確認
        # if step < 300 or step > (learningtime-300): #最初と最後の状態の保存
        #     car.recordhistory()
        #     alllight.append([Tlst[i].color for i in range(len(Tlst))])# [[g, r, r, r, r,...],...]など
        car.recordhistory()
        alllight.append([Tlst[i].color for i in range(len(Tlst))])
        if step == int((learningtime // 3) * 2):
            for j in Tlst: j.epsilon = 0
        if step % 50 == 0:
            print(step)

    for num, i in enumerate(Tlst): #Qテーブルの表示
        x = i.qtable.keys()
        x = sorted(x)
        print("↓agent{}_qtable".format(num))
        for j in x:
            print(j, list(map(rnd, i.qtable[j])))


    lst = []
    for i in range(lightnum):
        # print("{}_start_reward".format(i), sum(Tlst[i].rewardhistory[100:110]))
        # print("{}_end_reward".format(i), sum(Tlst[i].rewardhistory[-10:]))
        print("{}_end_reward".format(i), sum(Tlst[i].rewardhistory))
        lst.append(sum(Tlst[i].rewardhistory))
    print("allpenalty", sum(lst))
    print("")

def reset(car, Tlst):
    car.__init__(len(Tlst))
    for i in Tlst:
        i.secondinit()


def fielddisplay(startdisplay, enddisplay, graphdisplay, car, Tlst, alllight, displaytime, cooperationflag, namelst):
    if startdisplay:getanimation(car.allpos[0:100], car.allspos[0:100], alllight[0:100], fielddata.size, displaytime, cooperationflag, 0)
    if enddisplay:getanimation(car.allpos[-200:-1], car.allspos[-200:-1], alllight[-200:-1], fielddata.size, displaytime, cooperationflag, 1)
    # if graphdisplay:getgraph([Tlst[2].rewardhistory, Tlst[3].rewardhistory], [Tlst[2].rewardstep, Tlst[3].rewardstep], namelst[cooperationflag])


if __name__ == "__main__":
    #環境データベース
    gennum = 3 #車の発生数
    #エージェントデータベース
    cooperationflag = 1 #協調の有無
    learningflag = 1 #学習自体の有無
    timelength = 70 #1時刻の長さ(ステップ数)
    timescale = 10 #時刻の数
    learningtime1 = 3000 #学習回数(実行数は100×learningtime)
    learningtime2 = 1200 #推論回数
    simulationtime = 1 #シミュレーションの繰り返し回数
    actlst = [[1, 9], [2, 8], [3, 7], [4, 5], [5, 4], [7, 3], [8, 2], [9, 1]]
    #評価データベース
    startdisplay = 0  # ラーニング1回目のアニメーション
    enddisplay = 1 # ラーニング最終回のアニメーション
    graphdisplay = 0 #グラフ表示
    displaytime = 500 #アニメーションの1ステップの表示時間
    lightnum = fielddata.lightnum #信号の数


    for __ in range(simulationtime): #シミュレーションn回分
        car = Allcar(lightnum)
        Tlst = [Tlight(actlst) for _ in range(lightnum)]  # 信号の全インスタンスを格納
        lighthistory = []  # 車の位置の変遷すべて
        namelst = [["NO cooperation", "NO cooperation"], ["cooperation", "NO cooperation"]]
        alllight = []  # すべてのラーニングのすべてのラーニングの状況
        learning(gennum, lightnum, cooperationflag, learningflag, timelength, timescale, learningtime1, actlst,\
                 car, Tlst, lighthistory, alllight)
        for _ in range(1): #学習後の報酬確認
            reset(car, Tlst)
            learning(gennum, lightnum, cooperationflag, learningflag, timelength, timescale, learningtime2, actlst,\
                     car, Tlst, lighthistory, alllight)

        fielddisplay(startdisplay, enddisplay, graphdisplay, car, Tlst, alllight, displaytime,
                     cooperationflag, namelst)