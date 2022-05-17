from PIL import Image
import copy

lightnum = 16

def iter_pixels(img: Image.Image, color):
    for x in range(img.width):
        for y in range(img.height):
            c = img.getpixel((x, y))
            if c == color:
                yield [y, x]

# img: Image.Image = Image.open('genpos.bmp')
im = Image.open('pos.bmp')
lst = []

genpos = [] #↓→↑←,(0, 250, 0) - (0, 220, 0)、10刻み
dispos = [] #(255, 0, 0)
stopline = [] #(250, 0, 0) - (95, 0, 0)、5刻み
throughpos = [] #(250, 250, 0) - (250, 124, 0)、上下左右の順、2刻み
nearpos = [] #(0, 250, 250) - (0, 250, 95)、上下-左右の順、5刻み
intersection = [] #(250, 0, 250) - (250, 0, 100)、10刻み、交差点の左上
turnleftpos = [] #(0, 0, 250) - (0, 0, 220)
utyokupos = []
b = []
size = im.width


for i in iter_pixels(im, (255, 0, 0)):
    dispos.append(i)
for i in range(4):
    for j in iter_pixels(im, (0, 250 - 10 * i, 0)):
        lst = j + [i, 0]
        if i == 0:
            lst[0] -= 1
        elif i == 1:
            lst[1] -= 1
        elif i == 2:
            lst[0] += 1
        else:
            lst[1] += 1
        genpos.append(lst)
for i in range(lightnum * 2):
    stopline.append([])
    for j in iter_pixels(im, (250 - 5 * i, 0, 0)):
        stopline[i].append(j)
for i in range(lightnum * 4):
    for j in iter_pixels(im, (250, 250 - 2 * i, 0)):
        throughpos.append(j)
for i in range(lightnum * 2):
    nearpos.append([])
    for j in iter_pixels(im, (0, 250, 250 - 5 * i)):
        nearpos[i].append(j)
for i in range(lightnum):
    for j in iter_pixels(im, (250, 0, 250 - 10 * i)): #各信号機の左上を指定
        lst = [[j[0], j[1]+1], j, [j[0]+1, j[1]], [j[0]+1, j[1]+1]]
        lst2 = [[j[0]+1, j[1]+2], [j[0]-1, j[1]+1], [j[0], j[1]-1], [j[0]+2, j[1]]]
        intersection.append(j)
        for j in range(len(lst)):
            if i == 0:
                turnleftpos.append([])
            turnleftpos[j].append(lst[j])
        for j, k in zip(lst, lst2):
            utyokupos.append(j)
            b.append(k)
intersectionpos = [[i[0] + 0.5, i[1] + 0.5] for i in intersection]
turnrightpos = [turnleftpos[3], turnleftpos[0], turnleftpos[1], turnleftpos[2]]

bumppos = {tuple(utyokupos[i]):b[i] for i in range(len(utyokupos))}

congestionsignal = {1:[4, 0], 3:[1, 1], 5:[5, 0], 6:[0, 1], 7:[2, 1],
                    9:[6, 0], 10:[1, 1], 11:[3, 1], 13:[7, 0], 14:[2, 1],
                    16:[0, 0], 17:[8, 0], 19:[5, 1], 20:[1, 0], 21:[9, 0], 22:[4, 1], 23:[6, 1],
                    24:[2, 0], 25:[10, 0], 26:[5, 1], 27:[7, 1], 28:[3, 0], 29:[11, 0], 30:[6, 1],
                    32:[4, 0], 33:[12, 0], 35:[9, 1], 36:[5, 0], 37:[13, 0], 38:[8, 1], 39:[10, 1],
                    40:[6, 0], 41:[14, 0], 42:[9, 1], 43:[11, 1], 44:[7, 0], 45:[15, 0], 46:[10, 1],
                    48:[8, 0], 51:[13, 1], 52:[9, 0], 54:[12, 1], 55:[14, 1], 56:[10, 0],
                    58:[13, 1], 59:[15, 1], 60:[11, 0], 62:[14, 1]} #[シグナルを送る信号機、縦0横1]

# genpos2 = [[-1, 12, 0, 0], [-1, 25, 0, 0], [-1, 38, 0, 0], [-1, 51, 0, 0],
#            [63, 11, 2, 0], [63, 24, 2, 0], [63, 37, 2, 0], [63, 50, 2, 0],
#            [11, -1, 1, 5], [24, -1, 1, 5], [37, -1, 1, 5], [50, -1, 1, 5],
#            [12, 63, 3, 5], [25, 63, 3, 5], [38, 63, 3, 5], [51, 63, 3, 5]]
#
# genpos3 = [[-1, 12, 0, 5], [-1, 25, 0, 5], [-1, 38, 0, 5], [-1, 51, 0, 5],
#            [63, 11, 2, 5], [63, 24, 2, 5], [63, 37, 2, 5], [63, 50, 2, 5],
#            [11, -1, 1, 0], [24, -1, 1, 0], [37, -1, 1, 0], [50, -1, 1, 0],
#            [12, 63, 3, 0], [25, 63, 3, 0], [38, 63, 3, 0], [51, 63, 3, 0]]

genspe1 = genpos[0:4] + genpos[8:12]
genspe2 = genpos[4:8] + genpos[12:16]

if __name__ == "__main__":
    print(genspe1 + genspe2)







