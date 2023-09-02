import cv2
import numpy as np

file_name = 'f1.jpg'
search_posx = 100
search_posy = 100

#色を算出する関数
def color_judge(value):
    color_wheel=[
        'purplish red','red','yellowish red','reddish orange','orange',
        'yellowish orange','reddish yellow','yellow','greenish yellow',
        'yellow green','yellowish green','green','bluish green','blue green1',
        'blue green2','greenish blue','blue1','blue2','purplish blue','violet',
        'bluish purple','purple','reddish purple','red purple','purplish red'
    ]
    color_wheel2=[
        'pink','orange','yellow green','green','blue'
    ]
    color_wheel3=['red','red-orange','yellow-orange',
                'yellow','yellow-grren','green',
                'blue-green','green-blue','blue',
                'blue-violet','violet','red-violet','red']
    index_value=int(round(value/(180/(len(color_wheel)-1)),0))
    return color_wheel[index_value]

    
#選択した場所の色を出力する
img=cv2.imread(file_name,cv2.IMREAD_COLOR)
cv2.circle(img,center=(search_posx,search_posy),radius=10,color=(255,255,255),thickness=3)
cv2.imwrite('img_point_'+str(search_posx)+'_'+str(search_posy)+'.jpg',img)
imghsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
color=color_judge(imghsv[search_posy,search_posx,0])
print(color)

#選択した場所の色を確認するための画像を出力
check=np.zeros((256,256,3),np.uint8)
check[:,:]=imghsv[search_posy,search_posx]
check = cv2.cvtColor(check, cv2.COLOR_HSV2BGR)
cv2.imwrite('img_check_'+str(search_posx)+'_'+str(search_posy)+'.jpg',check)