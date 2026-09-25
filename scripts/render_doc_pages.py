import fitz

doc = fitz.open('documentação/Aula_2_Conceitos_especificacoes.pdf')
if doc.is_encrypted:
    doc.authenticate('')
print("Aula_2 pages:", len(doc))
doc[11].get_pixmap(dpi=150).save('artifacts/aula2_p12.png') # page 12
doc[12].get_pixmap(dpi=150).save('artifacts/aula2_p13.png') # page 13
print("Saved aula2_p12 and p13")

doc6r = fitz.open('documentação/Denavit_Hartenberg_robo_6R.pdf')
if doc6r.is_encrypted:
    doc6r.authenticate('')
print("6R pages:", len(doc6r))
for i in range(len(doc6r)):
    doc6r[i].get_pixmap(dpi=150).save(f'artifacts/robo6r_p{i+1}.png')
print("Saved robo6r pages")
