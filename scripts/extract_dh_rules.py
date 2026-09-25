import pypdf

reader = pypdf.PdfReader('documentação/Aula_DH.pdf')
for p in [5, 6, 7, 8, 9, 11, 21, 22, 23, 24, 25]: # 0-indexed: pages 6-10, 12, 22-26
    print(f"=== PAGE {p+1} ===")
    print(reader.pages[p].extract_text().strip())
