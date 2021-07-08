import os
import markdown
#from markdown.__meta__ import __version_info__
#from html import parser # unused but force pyinstaller to load 
from bs4 import BeautifulSoup

class Documentation:
    def __init__(self):
        self.docPath = os.path.dirname(os.path.abspath('docs.md'))


    def subsetManual(self, section):
        docuFile = os.path.join(self.docPath, 'docs.md')
        print(docuFile)
        html = markdown.markdown(open(docuFile).read())
        extractedSection = "".join(BeautifulSoup(html, features="html.parser").find(class_=section))
        print(extractedSection)

    def fullManual(self):
        docuFile = os.path.join(self.docPath, 'docs.md')
        html = markdown.markdown(open(docuFile).read())
        extractedDocText = "".join(BeautifulSoup(html, features="html.parser").findAll(text=True))
        print(extractedDocText)

