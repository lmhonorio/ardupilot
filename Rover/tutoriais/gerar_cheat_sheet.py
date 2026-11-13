#!/usr/bin/env python3
"""Render CHEAT_SHEET.md to a selectable, paginated PDF. Requires reportlab."""
import html
import re
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Flowable, PageBreak, Paragraph, SimpleDocTemplate, Spacer,
)

ROOT = Path(__file__).resolve().parent
FONT_DIR = Path('/usr/share/fonts/truetype/dejavu')
for name, file in [('Body', 'DejaVuSans.ttf'), ('Bold', 'DejaVuSans-Bold.ttf'),
                   ('Mono', 'DejaVuSansMono.ttf')]:
    pdfmetrics.registerFont(TTFont(name, str(FONT_DIR/file)))
pdfmetrics.registerFontFamily('Body', normal='Body', bold='Bold', italic='Body', boldItalic='Bold')
INK = colors.HexColor('#17324D')
TEAL = colors.HexColor('#007E87')
PAGE_W, PAGE_H = A4
MARGIN = 37
WIDTH = PAGE_W - 2*MARGIN
styles = {
    'h1': ParagraphStyle('h1', fontName='Bold', fontSize=17, leading=21,
                         textColor=INK, spaceAfter=12, keepWithNext=True),
    'h2': ParagraphStyle('h2', fontName='Bold', fontSize=10.7, leading=14,
                         textColor=TEAL, spaceBefore=8, spaceAfter=4, keepWithNext=True),
    'body': ParagraphStyle('body', fontName='Body', fontSize=9.3, leading=13.2,
                           textColor=INK, spaceAfter=6, alignment=TA_LEFT),
    'bullet': ParagraphStyle('bullet', fontName='Body', fontSize=9.3, leading=13.2,
                             textColor=INK, leftIndent=9, firstLineIndent=-7, spaceAfter=3),
}


def inline(text):
    """Small Markdown subset used by this command reference."""
    tokens = re.split(r'(`[^`]+`|\[[^\]]+\]\([^)]+\))', text)
    out = []
    for token in tokens:
        if token.startswith('`') and token.endswith('`'):
            out.append('<font name="Mono" size="8.6">'+html.escape(token[1:-1])+'</font>')
        elif re.fullmatch(r'\[[^\]]+\]\([^)]+\)', token):
            label, url = re.match(r'\[([^\]]+)\]\(([^)]+)\)', token).groups()
            out.append('<link href="'+html.escape(url, quote=True)+'" color="#007E87">'+html.escape(label)+'</link>')
        else:
            out.append(html.escape(token))
    return ''.join(out)


class Commands(Flowable):
    def __init__(self, lines):
        Flowable.__init__(self)
        self.lines = lines
        self.size = 8.8
        self.leading = 11.6
        self.height = len(lines)*self.leading + 14
        self.width = WIDTH
        for line in lines:
            actual = pdfmetrics.stringWidth(line, 'Mono', self.size)
            if actual > WIDTH - 18:
                raise ValueError('Linha de comando excede largura: '+line)

    def wrap(self, available_width, available_height):
        self.width = available_width
        return self.width, self.height

    def draw(self):
        c = self.canv
        c.setFillColor(colors.HexColor('#F0F4F7'))
        c.roundRect(0, 0, self.width, self.height, 4, fill=1, stroke=0)
        c.setFillColor(INK)
        c.setFont('Mono', self.size)
        y = self.height - 8 - self.size
        for line in self.lines:
            c.drawString(9, y, line)
            y -= self.leading


def page(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(TEAL)
    canvas.setLineWidth(1.5)
    canvas.line(MARGIN, PAGE_H-24, PAGE_W-MARGIN, PAGE_H-24)
    canvas.setFont('Bold', 7)
    canvas.setFillColor(INK)
    canvas.drawString(MARGIN, PAGE_H-17, 'ROVER  /  REFERÊNCIA DE COMANDOS')
    canvas.setFont('Body', 7)
    canvas.drawString(MARGIN, 22, '13/11/2026  •  ROS 2 Jazzy  •  comandos selecionáveis')
    canvas.drawRightString(PAGE_W-MARGIN, 22, f'{doc.page} / 12')
    canvas.restoreState()


def main():
    story = []
    paragraph = []
    code = None

    def flush():
        if paragraph:
            story.append(Paragraph(inline(' '.join(paragraph)), styles['body']))
            paragraph.clear()

    for line in (ROOT/'CHEAT_SHEET.md').read_text().splitlines():
        if line.startswith('```'):
            flush()
            if code is None:
                code = []
            else:
                story.extend([Commands(code), Spacer(1, 5)])
                code = None
            continue
        if code is not None:
            code.append(line)
            continue
        if line == '<!-- pagebreak -->':
            flush()
            story.append(PageBreak())
        elif line.startswith('# '):
            flush()
            story.append(Paragraph(inline(line[2:]), styles['h1']))
        elif line.startswith('## '):
            flush()
            story.append(Paragraph(inline(line[3:]), styles['h2']))
        elif line.startswith('- '):
            flush()
            story.append(Paragraph('• '+inline(line[2:]), styles['bullet']))
        elif not line.strip():
            flush()
        else:
            paragraph.append(line.strip())
    flush()
    if code is not None:
        raise ValueError('Bloco de código sem fechamento')
    target = ROOT/'CHEAT_SHEET.pdf'
    doc = SimpleDocTemplate(str(target), pagesize=A4, rightMargin=MARGIN,
                            leftMargin=MARGIN, topMargin=39, bottomMargin=39,
                            title='Rover — comandos de operação', author='Projeto Rover',
                            subject='ROS 2, simulação, Raspberry, missão, docking e MID360')
    doc.build(story, onFirstPage=page, onLaterPages=page)
    print(target)


if __name__ == '__main__':
    main()
