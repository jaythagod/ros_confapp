#!/usr/bin/env python
import os
import markdown
from bs4 import BeautifulSoup

class Documentation:
    def __init__(self):
        self.docPath = os.path.dirname(os.path.abspath('../docs/docs.md'))


    def subsetManual(self, section):
        docuFile = os.path.join(self.docPath, 'docs.md')
        html = markdown.markdown(open(docuFile).read())
        extractedSection = "".join(BeautifulSoup(html, 'html5lib').find(class_=section))
        print(extractedSection)

    def fullManual(self):
        docuFile = os.path.join(self.docPath, 'docs.md')
        html = markdown.markdown(open(docuFile).read())
        extractedDocText = "".join(BeautifulSoup(html, 'html5lib').findAll(text=True))
        print(extractedDocText)

