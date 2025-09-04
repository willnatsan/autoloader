INVALID_CARRIER_INDEX = [(col, row) for col in range(5, 7) for row in range(4, 12)]

slots = [
    [i, j] for i in range(12) for j in range(12) if (i, j) not in INVALID_CARRIER_INDEX
]

for slot in slots:
    print(f"- - {slot[0]}\n  - {slot[1]}")
