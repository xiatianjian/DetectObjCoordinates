import tkinter as tk
from tkinter import ttk
import tkinter.filedialog as dir
import subprocess
import os

class APPGui:
    imgLPath = ""
    imgRPath = ""
    def __init__(self):
        root = tk.Tk()
        self.createContent(root)
        root.title("物体运动轨迹检测系统V1.0")
        root.update()
        # 以下方法用来计算并设置窗体显示时，在屏幕中心居中
        curWidth = root.winfo_width()  # get current width
        curHeight = root.winfo_height()  # get current height
        scnWidth, scnHeight = root.maxsize()  # get screen width and height
        tmpcnf = '+%d+%d' % ((scnWidth - curWidth) / 2, (scnHeight - curHeight) / 2)
        root.geometry(tmpcnf)
        root.mainloop()

    def getListVal(self):
        return self.listVal
    def selectLeftImg(self):
        defaultDir = "D:\\"
        self.imgLPath = tk.filedialog.askopenfilename(title="选择文件",initialdir=(os.path.expanduser(defaultDir)))
        self.listVal.set(self.imgLPath)
    def getImgLPath(self):
        return self.imgLPath
    def selectRightImg(self):
        defaultDir = "D:\\"
        self.imgRPath = tk.filedialog.askopenfilename(title="选择文件",initialdir=(os.path.expanduser(defaultDir)))
        self.listVal.set(self.imgLPath)
    def getImgRPath(self):
        return self.imgRPath
    def createContent(self, root):
        lf = ttk.LabelFrame(root, text="物体运动轨迹检测系统V1.0")
        lf.pack(fill=tk.X, padx=15, pady=8)

        #创建按钮区
        buttonFrame = tk.Frame()
        buttonFrame.pack(fill=tk.BOTH,expand=tk.YES,sid=tk.TOP,padx=50,pady=8)
        #buttonFrame.grid(row=0,column=2)
        butMatch = tk.Button(buttonFrame,text="立体匹配",command=lambda:self.stereoMatch(self))
        butMatch.grid(row=0,column=0,sticky=tk.W)
        butDetect = tk.Button(buttonFrame,text="物体运动轨迹检测",command=lambda:self.trackDetect(self))
        butDetect.grid(row=0,column=1,sticky=tk.E)
        butl = tk.Button(buttonFrame,text="选择左相机文件",command=lambda:self.selectLeftImg())
        butl.grid(row=1,column=0,sticky=tk.W)
        butr = tk.Button(buttonFrame,text="选择右相机文件",command=lambda:self.selectRightImg())
        butr.grid(row=1, column=1, sticky=tk.E)

        #创建文本分区
        bottomFrame = tk.Frame()
        bottomFrame.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP, padx=50, pady=8)
        band = tk.Frame(bottomFrame)
        band.pack(fill=tk.BOTH, expand=tk.YES, side=tk.TOP)
        self.listVal = tk.StringVar()
        listbox = tk.Listbox(band, listvariable=self.listVal, height=18)
        listbox.pack(side=tk.LEFT, fill=tk.X, expand=tk.YES)

        vertical_bar = ttk.Scrollbar(band, orient=tk.VERTICAL, command=listbox.yview)
        vertical_bar.pack(side=tk.RIGHT, fill=tk.Y)
        listbox['yscrollcommand'] = vertical_bar.set

        horizontal_bar = ttk.Scrollbar(bottomFrame, orient=tk.HORIZONTAL, command=listbox.xview)
        horizontal_bar.pack(side=tk.BOTTOM, fill=tk.X)
        listbox['xscrollcommand'] = horizontal_bar.set

        self.listVal.set(("no text"))

    def stereoMatch(self,obj):
        #str里面是命令行
        str = "stereoMatch.exe"
        str1 = " "
        str1 += self.imgLPath
        str2 = " "
        str2 += self.imgRPath
        if self.imgLPath != "" and self.imgRPath != "":
            str += str1
            str += str2
        p = subprocess.Popen(str,
                             shell=True,
                             stdout=subprocess.PIPE)
        p.wait()
        coutRes = p.stdout.readlines()

        lst = list()
        for line in coutRes:
            lst.append(line)
        obj.getListVal().set(lst)

    def trackDetect(self,obj):
        # p = subprocess.Popen("varietyOfCoordOfObjs.exe",shell=True,
        #                         stdout=subprocess.PIPE)
        # p.wait()
        file = open("trackResults.txt","r")
        lst = list()
        for line in file:
            lst.append(line)
        obj.getListVal().set(lst)

if __name__ == "__main__":
    APPGui()