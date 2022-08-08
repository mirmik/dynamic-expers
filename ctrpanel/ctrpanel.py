#!/usr/bin/python3

import zengraph
import rxsignal
import pycrow
from pycrow.rxcrow import rxpublish
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *

pycrow.start_client()

sld0 = QSlider(Qt.Horizontal)
sld1 = QSlider(Qt.Horizontal)
sld2 = QSlider(Qt.Horizontal)

t = rxsignal.rxinterval(0.1)

s0 = t.map(lambda x: sld0.value())
s1 = t.map(lambda x: sld1.value())
s2 = t.map(lambda x: sld2.value())

z = rxsignal.zip(s0,s1,s2)

rxpublish("ctrpanel/sld0", z.map(lambda tpl: (" ".join([str(i) for i in tpl])+"\n").encode("utf-8")))

zengraph.flowplot(t, s0, s1, s2, yrange=(0,100))
zengraph.disp(sld0, 2, 1)
zengraph.disp(sld1, 3, 1)
zengraph.disp(sld2, 4, 1)
zengraph.show()