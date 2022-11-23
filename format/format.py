import fileinput
from shutil import copyfile

copyfile("in.txt", "out.txt")
i = 0
for line in fileinput.input(files=('out.txt'), inplace=True):
    newLine = "rom[" + str(i) + "] = " + "32'b" + line[0:-1] + ";"
    print(newLine)
    i += 1
fileinput.close()
