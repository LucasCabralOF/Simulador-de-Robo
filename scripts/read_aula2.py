import pypdf

reader = pypdf.PdfReader('documentação/Aula_2_Conceitos_especificacoes.pdf')
for p in range(10, 20):
    print(f"--- PAGE {p+1} ---")
    print(reader.pages[p].extract_text())
