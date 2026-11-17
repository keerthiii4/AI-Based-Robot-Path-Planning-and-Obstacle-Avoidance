import tkinter as tk

root = tk.Tk()
root.title("Robot Simulator Test")
root.geometry("400x200")

label = tk.Label(
    root,
    text="Robot GUI is working",
    font=("Arial", 16)
)
label.pack(pady=60)

root.mainloop()
