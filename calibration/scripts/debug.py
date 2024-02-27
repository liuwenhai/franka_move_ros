from tkinter import *

def record():
    print('Record')

def delete():
    print('Delete')

if __name__ == "__main__":
    root = Tk()
    root.title("Record joint position")
    root.geometry("300x75+1000+0")
    B1 = Button(root, text="Record",command=record)
    B1.place(x = 30, y = 20)
    
    B2 = Button(root, text="Delete",command=delete)
    B2.place(x = 160, y = 20)
    print('done1')
    root.mainloop()
    print('done')