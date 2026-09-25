import fitz

doc = fitz.open('documentação/Denavit_Hartenberg_robo_6R.pdf')
print("Needs pass:", doc.needs_pass)
print("Is encrypted:", doc.is_encrypted)
print("Auth empty:", doc.authenticate(''))
