import sys

for mod in ['pypdf', 'fitz', 'pdfplumber', 'pypdf2']:
    try:
        __import__(mod)
        print('Available:', mod)
    except ImportError:
        pass
