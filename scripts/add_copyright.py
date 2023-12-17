#! /usr/bin/env python3

# MIT License
#
# Copyright (c) 2023 Carnegie Mellon University
#
# This file is part of RoWild.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys
from pathlib import Path
from scriptlib import isDontCareFile
import re
import os

MAXIMUM_CHARS = 80

LICENSE_MESSAGE = """MIT License

Copyright (c) 2023 Carnegie Mellon University

This file is part of RoWild.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""


def splitCommentLicense(commentSymbol):
    out = ""
    for line in LICENSE_MESSAGE.splitlines():
        out += commentSymbol
        for word in line.split():
            if len(out.splitlines()[-1]) + 1 + len(word) > MAXIMUM_CHARS:
                out += "\n"
                out += commentSymbol
            out += " " + word
        out += "\n"

    return out



def addMessage(filePath, extension, msg):
    f = open(filePath, 'r+')
    content = f.readlines()
    shebang = None
    if content[0].startswith("#!"):
        shebang = content[0]
    else:
        pass
        # if extension not in ['.cpp', '.hpp', '.c', '.h']:
            # print(f'{filePath} doesn\'t have a shebang')

    f.seek(0, 0)
    if shebang: f.write(shebang + '\n')
    f.write(msg)

    firstLine = 1 if shebang else 0
    if not content[firstLine].isspace(): f.write('\n')
    f.writelines(content[firstLine:])



def generateMessage(extension):
    out = ''
    if extension in ['.cpp', '.hpp', '.c', '.h', '.v']:
        out += '/*\n'
        out += splitCommentLicense(' *')
        out += '*/\n'
    elif extension in ['.sh', '.py', '.tcl']:
        out += splitCommentLicense('#')
    else:
        assert False, f"Unknown extension: {extension}"

    return out



def containsCopyright(filePath):
    f = open(filePath, 'r')
    content = f.read()
    f.close()

    if "Copyright (c)" in content or "Copyright (C)" in content:
        return "yes"

    if (content.startswith("/*") or content.startswith("# ")) and os.path.splitext(filePath)[1] != '.tcl':
        return "not sure"

    if "copyright" in content:
        return "not sure"

    return "no"


def isItMyself(filePath):
    return Path(filePath) == Path(__file__)



if __name__ == '__main__':
    if len(sys.argv[1:]) != 1:
        print('Missing parent path')
        exit(1)

    modifiedFiles = []
    exts = ['.cpp', '.hpp', '.c', '.h', '.sh', '.py', '.cu', '.v', '.tcl']
    for e in exts:
        allFiles = list(Path(sys.argv[1]).glob('**/*'+e))
        for filePath in allFiles:
            if not isDontCareFile(filePath) and not isItMyself(filePath):
                if containsCopyright(filePath).lower() == "no":
                    msg = generateMessage(e)
                    addMessage(filePath, e, msg)
                    modifiedFiles.append(filePath)
                elif containsCopyright(filePath).lower() == "yes":
                    print(f'{filePath} already has a copyright notice')
                else:
                    print(f'Not sure about {filePath}. Check it manually')
            else:
                print(f'{filePath} is a don\'t care file')

    print('-' * 20)
    print(f"The copyright message was added to the following {len(modifiedFiles)} files:")
    print('\n'.join([str(f) for f in modifiedFiles]))
