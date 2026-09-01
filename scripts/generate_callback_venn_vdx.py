#!/usr/bin/env python3
"""Generate an editable Visio XML Drawing (.vdx) for the ROS data-race Venn figure."""

from pathlib import Path
import xml.etree.ElementTree as ET


NS = "http://schemas.microsoft.com/visio/2003/core"
ET.register_namespace("", NS)


def el(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, f"{{{NS}}}{tag}", attrs)
    if text is not None:
        node.text = str(text)
    return node


def add_xform(shape, pin_x, pin_y, width, height):
    xform = el(shape, "XForm")
    el(xform, "PinX", f"{pin_x:.4f}")
    el(xform, "PinY", f"{pin_y:.4f}")
    el(xform, "Width", f"{width:.4f}")
    el(xform, "Height", f"{height:.4f}")
    el(xform, "LocPinX", f"{width / 2:.4f}", F="Width*0.5")
    el(xform, "LocPinY", f"{height / 2:.4f}", F="Height*0.5")
    el(xform, "Angle", "0")


def add_rect_geom(shape, width, height, no_fill=True, no_line=True):
    geom = el(shape, "Geom", IX="0")
    el(geom, "NoFill", "1" if no_fill else "0")
    el(geom, "NoLine", "1" if no_line else "0")
    move = el(geom, "MoveTo", IX="1")
    el(move, "X", "0")
    el(move, "Y", "0")
    points = [
        ("Width", "0"),
        ("Width", "Height"),
        ("0", "Height"),
        ("0", "0"),
    ]
    for ix, (x, y) in enumerate(points, start=2):
        row = el(geom, "LineTo", IX=str(ix))
        el(row, "X", str(width if x == "Width" else 0), F=x)
        el(row, "Y", str(height if y == "Height" else 0), F=y)


def add_ellipse(shapes, shape_id, name, pin_x, pin_y, width, height,
                fill_color=None, fill_trans=0.0, line_color=None, line_weight=0.0304):
    shape = el(shapes, "Shape", ID=str(shape_id), NameU=name, Name=name, Type="Shape")
    add_xform(shape, pin_x, pin_y, width, height)

    line = el(shape, "Line")
    if line_color:
        el(line, "LineWeight", f"{line_weight:.4f}", Unit="PT")
        el(line, "LineColor", line_color)
        el(line, "LinePattern", "1")
    else:
        el(line, "LinePattern", "0")

    fill = el(shape, "Fill")
    if fill_color:
        el(fill, "FillForegnd", fill_color)
        el(fill, "FillForegndTrans", f"{fill_trans:.4f}")
        el(fill, "FillPattern", "1")
    else:
        el(fill, "FillPattern", "0")

    geom = el(shape, "Geom", IX="0")
    el(geom, "NoFill", "0" if fill_color else "1")
    el(geom, "NoLine", "0" if line_color else "1")
    ellipse = el(geom, "Ellipse", IX="1")
    el(ellipse, "X", f"{width / 2:.4f}", F="Width*0.5")
    el(ellipse, "Y", f"{height / 2:.4f}", F="Height*0.5")
    el(ellipse, "A", f"{width:.4f}", F="Width")
    el(ellipse, "B", f"{height / 2:.4f}", F="Height*0.5")
    el(ellipse, "C", f"{width / 2:.4f}", F="Width*0.5")
    el(ellipse, "D", f"{height:.4f}", F="Height")


def add_text(shapes, shape_id, name, pin_x, pin_y, width, height, text,
             size_pt, color="#111111", style=0, align=1, runs=None):
    shape = el(shapes, "Shape", ID=str(shape_id), NameU=name, Name=name, Type="Shape")
    add_xform(shape, pin_x, pin_y, width, height)

    line = el(shape, "Line")
    el(line, "LinePattern", "0")
    fill = el(shape, "Fill")
    el(fill, "FillPattern", "0")

    block = el(shape, "TextBlock")
    el(block, "VerticalAlign", "1")
    el(block, "TxtMarginLeft", "0")
    el(block, "TxtMarginRight", "0")
    el(block, "TxtMarginTop", "0")
    el(block, "TxtMarginBottom", "0")

    if runs is None:
        runs = [(text, style, color)]
    for ix, (_, run_style, run_color) in enumerate(runs):
        char = el(shape, "Char", IX=str(ix))
        el(char, "Font", "0")
        el(char, "Color", run_color)
        el(char, "Style", str(run_style))
        el(char, "Size", f"{size_pt / 72:.6f}", Unit="PT")

    para = el(shape, "Para", IX="0")
    el(para, "HorzAlign", str(align))
    el(para, "SpLine", "-1")

    add_rect_geom(shape, width, height)
    text_node = el(shape, "Text")
    for ix, (segment, _, _) in enumerate(runs):
        cp = el(text_node, "cp", IX=str(ix))
        if ix == 0:
            pp = el(text_node, "pp", IX="0")
            pp.tail = segment
        else:
            cp.tail = segment


def build(output_path):
    root = ET.Element(
        f"{{{NS}}}VisioDocument",
        {"version": "14.0", "metric": "0", "DocLangID": "1033", "xml:space": "preserve"},
    )

    props = el(root, "DocumentProperties")
    el(props, "Title", "Number of Data Races Reported by ROS Open-Source Project Communities")
    el(props, "Creator", "OpenAI Codex")
    el(props, "Description", "Editable Venn diagram of reported ROS data-race concurrency categories")

    fonts = el(root, "Fonts")
    font = el(fonts, "Font", ID="0")
    el(font, "Name", "Calibri")
    el(font, "CharSet", "0")

    styles = el(root, "StyleSheets")
    el(styles, "StyleSheet", ID="0", NameU="No Style", Name="No Style")

    pages = el(root, "Pages")
    page = el(pages, "Page", ID="0", NameU="Page-1", Name="Page-1", ViewScale="1")
    page_sheet = el(page, "PageSheet")
    page_props = el(page_sheet, "PageProps")
    el(page_props, "PageWidth", "11.2")
    el(page_props, "PageHeight", "6.1")
    el(page_props, "PageScale", "1", Unit="IN")
    el(page_props, "DrawingScale", "1", Unit="IN")
    el(page_props, "DrawingSizeType", "3")
    el(page_props, "DrawingScaleType", "0")

    shapes = el(page, "Shapes")
    sid = 1

    ellipses = [
        ("Callback fill", 2.410, 3.354, "#DCE6F1"),
        ("Worker Thread fill", 4.690, 3.354, "#E7E6E6"),
        ("Lifecycle fill", 3.550, 2.024, "#EBF1DE"),
    ]
    for name, x, y, color in ellipses:
        add_ellipse(shapes, sid, name, x, y, 3.800, 2.888, fill_color=color, fill_trans=0.32)
        sid += 1
    for name, x, y, _ in ellipses:
        add_ellipse(shapes, sid, name.replace("fill", "outline"), x, y, 3.800, 2.888,
                    line_color="#4A4A4A")
        sid += 1

    add_text(shapes, sid, "Title", 5.600, 5.720, 10.4, 0.42,
             "Number of Data Races Reported by ROS Open-Source Project Communities",
             18, style=1)
    sid += 1

    labels = [
        ("Callback label", 2.030, 4.228, 2.20, "A: Callback", "#0070C0"),
        ("Worker Thread label", 5.070, 4.228, 2.75, "B: Worker Thread", "#111111"),
        ("Lifecycle label", 3.550, 1.080, 2.20, "C: Lifecycle", "#111111"),
    ]
    for name, x, y, width, text, color in labels:
        add_text(shapes, sid, name, x, y, width, 0.34, text, 16.5, color=color, style=3)
        sid += 1

    counts = [
        ("Callback only", 1.422, 3.354, "54"),
        ("Worker Thread only", 5.678, 3.354, "7"),
        ("Lifecycle only", 3.550, 1.780, "5"),
        ("Callback and Worker Thread", 3.550, 4.228, "60"),
        ("Callback and Lifecycle", 2.524, 2.138, "21"),
        ("Worker Thread and Lifecycle", 4.576, 2.138, "0"),
        ("All three", 3.550, 2.860, "0"),
    ]
    for name, x, y, text in counts:
        add_text(shapes, sid, name, x, y, 0.55, 0.38, text, 18.5, style=1)
        sid += 1

    equations = [
        (4.200, "A", "36.7%"),
        (3.750, "B", "4.8%"),
        (3.300, "C", "3.4%"),
        (2.850, "A + B", "40.8%"),
        (2.400, "A + C", "14.3%"),
        (1.950, "B + C", "0.0%"),
        (1.500, "A + B + C", "0.0%"),
    ]
    for index, (y, expression, value) in enumerate(equations, start=1):
        add_text(
            shapes,
            sid,
            f"Region proportion {index}",
            9.000,
            y,
            3.200,
            0.42,
            "",
            17,
            align=1,
            runs=[
                (expression, 2, "#111111"),
                (f" = {value}", 0, "#111111"),
            ],
        )
        sid += 1

    ET.indent(root, space="  ")
    tree = ET.ElementTree(root)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    output = Path(__file__).resolve().parents[1] / "outputs" / "callback_venn_visio_style.vdx"
    build(output)
    print(output)
