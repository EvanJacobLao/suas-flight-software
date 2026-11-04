from pathlib import Path

# Find the project root using this script's location
project_root = Path(__file__).resolve().parents[2]
source = project_root / "datasets/suas_v1"
destination = project_root / "datasets/suas_v1_detection"

for split in ("train", "valid", "test"):
    source_labels = source / split / "labels"
    output_labels = destination / split / "labels"
    output_labels.mkdir(parents=True, exist_ok=True)

    for annotation in source_labels.glob("*.txt"):
        converted_rows = []

        for line in annotation.read_text().splitlines():
            parts = line.split()

            if not parts:
                continue

            if len(parts) == 5:
                # Already a bounding box: keep it unchanged
                converted_rows.append(line)
                continue

            if len(parts) < 7 or (len(parts) - 1) % 2 != 0:
                raise ValueError(f"Invalid polygon in {annotation.name}")

            class_id = parts[0]
            coordinates = [float(value) for value in parts[1:]]

            xs = coordinates[0::2]
            ys = coordinates[1::2]

            centre_x = (min(xs) + max(xs)) / 2
            centre_y = (min(ys) + max(ys)) / 2
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)

            converted_rows.append(
                f"{class_id} {centre_x:.8f} {centre_y:.8f} "
                f"{width:.8f} {height:.8f}"
            )

        # annotation.name preserves the original filename
        output_file = output_labels / annotation.name
        contents = "\n".join(converted_rows)

        if converted_rows:
            contents += "\n"

        output_file.write_text(contents, encoding="utf-8")

    print(f"Saved {split} labels to {output_labels}")

for split in ("train", "valid", "test"):
    source_images = source / split / "images"