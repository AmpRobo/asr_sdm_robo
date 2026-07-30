#!/usr/bin/env python3
"""
Clean SVG renderer: parses our SVG, renders with cairo, outputs PNG.
"""
import cairo
import math
import re
from lxml import etree

def hex_to_rgb(h):
    h = h.lstrip('#')
    if len(h) == 3:
        h = ''.join(c*2 for c in h)
    # Pad to 6 chars
    h = (h + '000000')[:6]
    return (int(h[0:2], 16)/255.0, int(h[2:4], 16)/255.0, int(h[4:6], 16)/255.0)

def parse_num(s, default=0.0):
    try:
        return float(s)
    except:
        return default

def parse_color(c, default=None):
    if not c or c in ('none', 'transparent'):
        return None
    if c.startswith('#'):
        return hex_to_rgb(c)
    return default

def set_source(ctx, color):
    if color:
        ctx.set_source_rgb(*color)

class SVGRender:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, int(w), int(h))
        self.ctx = cairo.Context(self.surface)
        # White background
        self.ctx.set_source_rgb(1, 1, 1)
        self.ctx.paint()
        self.ctx.set_line_cap(cairo.LINE_CAP_ROUND)
        self.ctx.set_line_join(cairo.LINE_JOIN_ROUND)

    def set_font(self, size, bold=False, italic=False):
        weight = cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL
        slant = cairo.FONT_SLANT_ITALIC if italic else cairo.FONT_SLANT_NORMAL
        self.ctx.select_font_face("Arial", slant, weight)
        self.ctx.set_font_size(size)

    def draw_rect(self, x, y, w, h, rx=0, ry=0,
                  fill=None, stroke=None, sw=1, opacity=1.0, dash=None):
        ctx = self.ctx
        r = min(rx or 0, w/2, h/2)
        if r > 0:
            ctx.move_to(x+r, y)
            ctx.line_to(x+w-r, y)
            ctx.arc(x+w-r, y+r, r, -math.pi/2, 0)
            ctx.line_to(x+w, y+h-r)
            ctx.arc(x+w-r, y+h-r, r, 0, math.pi/2)
            ctx.line_to(x+r, y+h)
            ctx.arc(x+r, y+h-r, r, math.pi/2, math.pi)
            ctx.line_to(x, y+r)
            ctx.arc(x+r, y+r, r, math.pi, -math.pi/2)
            ctx.close_path()
        else:
            ctx.rectangle(x, y, w, h)

        did_fill = False
        if fill:
            set_source(ctx, fill)
            ctx.fill()
            did_fill = True
            ctx.new_path()

        if stroke:
            if r > 0:
                # Redraw border shape
                if r > 0:
                    ctx.move_to(x+r, y)
                    ctx.line_to(x+w-r, y)
                    ctx.arc(x+w-r, y+r, r, -math.pi/2, 0)
                    ctx.line_to(x+w, y+h-r)
                    ctx.arc(x+w-r, y+h-r, r, 0, math.pi/2)
                    ctx.line_to(x+r, y+h)
                    ctx.arc(x+r, y+h-r, r, math.pi/2, math.pi)
                    ctx.line_to(x, y+r)
                    ctx.arc(x+r, y+r, r, math.pi, -math.pi/2)
                    ctx.close_path()
                else:
                    ctx.rectangle(x, y, w, h)
            set_source(ctx, stroke)
            ctx.set_line_width(sw)
            if dash:
                ctx.set_dash(dash)
            ctx.stroke()
            ctx.set_dash([])
            ctx.new_path()
        elif not did_fill:
            ctx.new_path()

    def draw_line(self, x1, y1, x2, y2, stroke='#000', sw=1, dash=None, marker_end=None):
        ctx = self.ctx
        color = stroke if isinstance(stroke, tuple) else parse_color(stroke)
        set_source(ctx, color)
        ctx.set_line_width(sw)
        if dash:
            ctx.set_dash(dash)
        ctx.move_to(x1, y1)
        ctx.line_to(x2, y2)
        ctx.stroke()
        ctx.set_dash([])
        ctx.new_path()

    def draw_text(self, text, x, y, anchor='start',
                   font_size=12, bold=False, italic=False,
                   color=(0,0,0), line_h=1.3):
        ctx = self.ctx
        self.set_font(font_size, bold, italic)
        ctx.set_source_rgb(*color)
        lines = text.strip().split('\n')
        bbox = ctx.text_extents('Ay')
        lh = (bbox[3] - bbox[1]) * line_h

        for i, line in enumerate(lines):
            ly = y + i * lh
            ext = ctx.text_extents(line)
            if anchor == 'middle':
                tx = x - (ext.width / 2 + ext.x_bearing)
            elif anchor == 'end':
                tx = x - (ext.width + ext.x_bearing)
            else:
                tx = x
            ctx.move_to(tx, ly)
            ctx.show_text(line)
        ctx.new_path()

    def draw_text_multiline(self, lines, x, y, anchor='start',
                             font_size=11, bold=False, color=(0.2,0.2,0.2)):
        ctx = self.ctx
        self.set_font(font_size, bold)
        ctx.set_source_rgb(*color)
        bbox = ctx.text_extents('Ay')
        lh = (bbox[3] - bbox[1]) * 1.35
        for i, line in enumerate(lines):
            ly = y + i * lh
            ext = ctx.text_extents(line)
            if anchor == 'middle':
                tx = x - (ext.width / 2 + ext.x_bearing)
            elif anchor == 'end':
                tx = x - (ext.width + ext.x_bearing)
            else:
                tx = x
            ctx.move_to(tx, ly)
            ctx.show_text(line)
        ctx.new_path()

    def draw_circle(self, cx, cy, r, fill=None, stroke=None, sw=1):
        ctx = self.ctx
        ctx.arc(cx, cy, r, 0, 2*math.pi)
        if fill:
            set_source(ctx, fill); ctx.fill_preserve()
        if stroke:
            set_source(ctx, stroke); ctx.set_line_width(sw); ctx.stroke()
        ctx.new_path()

    def draw_polygon(self, points, fill=None, stroke=None, sw=1):
        ctx = self.ctx
        pts = [float(v) for v in re.findall(r'-?\d+\.?\d*', points)]
        if len(pts) < 4:
            return
        ctx.move_to(pts[0], pts[1])
        for i in range(2, len(pts), 2):
            ctx.line_to(pts[i], pts[i+1])
        ctx.close_path()
        if fill:
            set_source(ctx, fill); ctx.fill_preserve()
        if stroke:
            set_source(ctx, stroke); ctx.set_line_width(sw); ctx.stroke()
        ctx.new_path()

    def draw_path(self, d, fill=None, stroke=None, sw=1, fill_rule=cairo.FILL_RULE_WINDING):
        ctx = self.ctx
        self._path_d(d)
        if fill:
            set_source(ctx, fill); ctx.fill_preserve(fill_rule)
        if stroke:
            set_source(ctx, stroke); ctx.set_line_width(sw); ctx.stroke()
        ctx.new_path()
        ctx.set_fill_rule(cairo.FILL_RULE_WINDING)

    def _path_d(self, d):
        ctx = self.ctx
        tokens = re.findall(r'[MmZzLlHhVvCcSsQqTtAa]| -?\d+\.?\d*(?:[eE][-+]?\d+)?', d)
        i = 0
        cx, cy = 0, 0
        sx, sy = 0, 0
        while i < len(tokens):
            t = tokens[i]; i += 1
            if not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', t):
                continue
            c = t
            if c == 'M':
                x = parse_num(tokens[i]); y = parse_num(tokens[i+1]); i += 2
                ctx.move_to(x, y); cx, cy = x, y; sx, sy = x, y
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    x = parse_num(tokens[i]); y = parse_num(tokens[i+1]); i += 2
                    ctx.line_to(x, y); cx, cy = x, y
            elif c == 'm':
                x = parse_num(tokens[i])+cx; y = parse_num(tokens[i+1])+cy; i += 2
                ctx.move_to(x, y); cx, cy = x, y; sx, sy = x, y
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    x = parse_num(tokens[i])+cx; y = parse_num(tokens[i+1])+cy; i += 2
                    ctx.line_to(x, y); cx, cy = x, y
            elif c == 'L':
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    x = parse_num(tokens[i]); y = parse_num(tokens[i+1]); i += 2
                    ctx.line_to(x, y); cx, cy = x, y
            elif c == 'l':
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    x = parse_num(tokens[i])+cx; y = parse_num(tokens[i+1])+cy; i += 2
                    ctx.line_to(x, y); cx, cy = x, y
            elif c == 'H':
                x = parse_num(tokens[i]); i += 1
                ctx.line_to(x, cy); cx = x
            elif c == 'h':
                x = parse_num(tokens[i])+cx; i += 1
                ctx.line_to(x, cy); cx = x
            elif c == 'V':
                y = parse_num(tokens[i]); i += 1
                ctx.line_to(cx, y); cy = y
            elif c == 'v':
                y = parse_num(tokens[i])+cy; i += 1
                ctx.line_to(cx, y); cy = y
            elif c in ('Z', 'z'):
                ctx.close_path(); cx, cy = sx, sy
            elif c == 'C':
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    x1 = parse_num(tokens[i]); y1 = parse_num(tokens[i+1])
                    x2 = parse_num(tokens[i+2]); y2 = parse_num(tokens[i+3])
                    x  = parse_num(tokens[i+4]); y  = parse_num(tokens[i+5])
                    i += 6
                    ctx.curve_to(x1, y1, x2, y2, x, y)
                    cx, cy = x, y
            elif c == 'c':
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    dx1 = parse_num(tokens[i]); dy1 = parse_num(tokens[i+1])
                    dx2 = parse_num(tokens[i+2]); dy2 = parse_num(tokens[i+3])
                    dx  = parse_num(tokens[i+4]); dy  = parse_num(tokens[i+5])
                    i += 6
                    ctx.curve_to(cx+dx1, cy+dy1, cx+dx2, cy+dy2, cx+dx, cy+dy)
                    cx += dx; cy += dy
            elif c in ('S', 's'):
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    x2 = parse_num(tokens[i]); y2 = parse_num(tokens[i+1])
                    x  = parse_num(tokens[i+2]); y  = parse_num(tokens[i+3])
                    i += 4
                    if c == 's': x += cx; y += cy; x2 += cx; y2 += cy
                    ctx.curve_to(cx, cy, x2, y2, x, y)
                    cx, cy = x, y
            elif c == 'A':
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    rx = parse_num(tokens[i]); ry = parse_num(tokens[i+1])
                    x_rot = parse_num(tokens[i+2])
                    large = int(tokens[i+3]); sweep = int(tokens[i+4])
                    x = parse_num(tokens[i+5]); y = parse_num(tokens[i+6])
                    i += 7
                    ctx.line_to(x, y); cx, cy = x, y
            elif c == 'a':
                while i < len(tokens) and not re.match(r'^[MmZzLlHhVvCcSsQqTtAa]', tokens[i]):
                    rx = parse_num(tokens[i]); ry = parse_num(tokens[i+1])
                    x_rot = parse_num(tokens[i+2])
                    large = int(tokens[i+3]); sweep = int(tokens[i+4])
                    dx = parse_num(tokens[i+5]); dy = parse_num(tokens[i+6])
                    i += 7
                    ctx.line_to(cx+dx, cy+dy)
                    cx += dx; cy += dy

    def draw_arrow(self, x1, y1, x2, y2, color='#1B4F72', sw=2):
        ctx = self.ctx
        c = color if isinstance(color, tuple) else parse_color(color)
        set_source(ctx, c)
        ctx.set_line_width(sw)
        ctx.move_to(x1, y1)
        ctx.line_to(x2, y2)
        ctx.stroke()
        # Arrowhead
        dx, dy = x2-x1, y2-y1
        L = math.sqrt(dx*dx+dy*dy)
        if L > 0:
            dx, dy = dx/L, dy/L
            aw = 10
            ax1 = x2 - dx*aw - dy*aw*0.5
            ay1 = y2 - dy*aw + dx*aw*0.5
            ax2 = x2 - dx*aw + dy*aw*0.5
            ay2 = y2 - dy*aw - dx*aw*0.5
            ctx.move_to(ax1, ay1)
            ctx.line_to(x2, y2)
            ctx.line_to(ax2, ay2)
            ctx.close_path()
            ctx.fill()
        ctx.new_path()

    def save(self, path):
        self.surface.write_to_png(path)


# ---- Render the SVG ----
def render_svg(svg_path, png_path):
    tree = etree.parse(svg_path)
    root = tree.getroot()
    NS = root.tag.split('}')[0].lstrip('{') if '}' in root.tag else ''

    def qn(tag):
        return f'{{{NS}}}{tag}' if NS else tag

    vb = root.get('viewBox', '').split()
    W = float(root.get('width', 1920))
    H = float(root.get('height', 1080))
    if len(vb) == 4:
        W, H = float(vb[2]), float(vb[3])

    print(f"Rendering {W}x{H}")

    r = SVGRender(W, H)

    # Color constants
    BLUE_DARK  = (0.106, 0.31, 0.447)     # #1B4F72
    BLUE_MED   = (0.16, 0.51, 0.72)       # #2980B9
    BLUE_LIGHT = (0.68, 0.84, 0.96)       # #AED6F1
    GREEN_DARK = (0.12, 0.69, 0.31)       # #1E8449
    GREEN_MED  = (0.15, 0.68, 0.38)       # #27AE60
    GREEN_LIGHT= (0.92, 0.97, 0.91)      # #EBF7EE
    RED_DARK   = (0.57, 0.15, 0.14)       # #922B21
    RED_MED    = (0.75, 0.19, 0.17)       # #C0392B
    RED_LIGHT  = (0.99, 0.93, 0.93)      # #FDEDEC
    ORANGE_MED = (0.90, 0.49, 0.13)      # #E67E22
    YELLOW     = (1.0, 0.98, 0.84)        # #FEFAE7
    WHITE      = (1.0, 1.0, 1.0)
    GRAY_DARK  = (0.2, 0.2, 0.2)
    GRAY_MED   = (0.4, 0.4, 0.4)
    GRAY_LIGHT = (0.53, 0.53, 0.53)
    GRAY_LINE  = (0.67, 0.67, 0.67)

    def text_elem(e, r):
        """Render a <text> SVG element."""
        x = parse_num(e.get('x', '0'))
        y = parse_num(e.get('y', '0'))
        anchor = e.get('text-anchor', 'start')
        fs = parse_num(e.get('font-size', '12'))
        bold = e.get('font-weight', 'normal') in ('bold', '700', '800', '900')
        italic = e.get('font-style', 'normal') == 'italic'
        fill = parse_color(e.get('fill', '#000'), GRAY_DARK)

        lines = []
        def collect_text(node):
            if node.text:
                lines.append(node.text)
            for child in node:
                collect_text(child)
                if child.tail:
                    lines.append(child.tail)
        collect_text(e)

        full_text = ''.join(lines).strip()
        if not full_text:
            return
        r.draw_text(full_text, x, y, anchor, fs, bold, italic, fill)

    def process_elem(e, r):
        # Skip comments and processing instructions
        if isinstance(e, (etree._Comment, etree._ProcessingInstruction)):
            return
        tag = e.tag
        if callable(tag):
            return  # lxml extended element
        tag = tag.split('}')[-1] if '}' in tag else tag

        if tag == 'rect':
            x = parse_num(e.get('x', '0'))
            y = parse_num(e.get('y', '0'))
            w = parse_num(e.get('width', '0'))
            h = parse_num(e.get('height', '0'))
            rx = parse_num(e.get('rx', '0'))
            ry = parse_num(e.get('ry', rx))
            fill = parse_color(e.get('fill', None))
            stroke = parse_color(e.get('stroke', None))
            sw = parse_num(e.get('stroke-width', '1'))
            dash = [float(v) for v in re.split(r'[,\s]+', e.get('stroke-dasharray', '').strip()) if v] if e.get('stroke-dasharray') else None
            r.draw_rect(x, y, w, h, rx, ry, fill, stroke, sw, dash=dash)

        elif tag == 'line':
            x1 = parse_num(e.get('x1', '0'))
            y1 = parse_num(e.get('y1', '0'))
            x2 = parse_num(e.get('x2', '0'))
            y2 = parse_num(e.get('y2', '0'))
            stroke = parse_color(e.get('stroke', '#000'))
            sw = parse_num(e.get('stroke-width', '1'))
            dash = [float(v) for v in re.split(r'[,\s]+', e.get('stroke-dasharray', '').strip()) if v] if e.get('stroke-dasharray') else None
            r.draw_line(x1, y1, x2, y2, stroke, sw, dash)

        elif tag == 'text':
            text_elem(e, r)

        elif tag == 'circle':
            cx = parse_num(e.get('cx', '0'))
            cy = parse_num(e.get('cy', '0'))
            rad = parse_num(e.get('r', '0'))
            fill = parse_color(e.get('fill', None))
            stroke = parse_color(e.get('stroke', None))
            sw = parse_num(e.get('stroke-width', '1'))
            r.draw_circle(cx, cy, rad, fill, stroke, sw)

        elif tag == 'polygon':
            pts = e.get('points', '')
            fill = parse_color(e.get('fill', None))
            stroke = parse_color(e.get('stroke', None))
            sw = parse_num(e.get('stroke-width', '1'))
            r.draw_polygon(pts, fill, stroke, sw)

        elif tag == 'path':
            d = e.get('d', '')
            fill = parse_color(e.get('fill', None))
            stroke = parse_color(e.get('stroke', None))
            sw = parse_num(e.get('stroke-width', '1'))
            r.draw_path(d, fill, stroke, sw)

        elif tag == 'g':
            transform = e.get('transform', '')
            if transform:
                r.ctx.save()
                _apply_transform(r.ctx, transform)
            for child in e:
                process_elem(child, r)
            if transform:
                r.ctx.restore()

        elif tag == 'defs':
            pass  # markers are defined but applied by refs

    def _apply_transform(ctx, transform):
        parts = [p.strip() + ')' for p in transform.replace(')', ' ),').split(',')]
        for part in parts:
            part = part.strip()
            if not part: continue
            if 'translate' in part:
                nums = re.findall(r'-?\d+\.?\d*', part)
                ctx.translate(float(nums[0]), float(nums[1]) if len(nums) > 1 else 0)
            elif 'scale' in part:
                nums = re.findall(r'-?\d+\.?\d*', part)
                ctx.scale(float(nums[0]), float(nums[1]) if len(nums) > 1 else float(nums[0]))
            elif 'rotate' in part:
                nums = re.findall(r'-?\d+\.?\d*', part)
                a = math.radians(float(nums[0]))
                if len(nums) >= 3:
                    ctx.translate(float(nums[1]), float(nums[2]))
                    ctx.rotate(a)
                    ctx.translate(-float(nums[1]), -float(nums[2]))
                else:
                    ctx.rotate(a)
            elif 'matrix' in part:
                nums = re.findall(r'-?\d+\.?\d*', part)
                m = [float(x) for x in nums]
                if len(m) == 6:
                    ctx.transform(cairo.Matrix(m[0], m[1], m[2], m[3], m[4], m[5]))

    # Process all elements in order
    for elem in root:
        process_elem(elem, r)

    r.save(png_path)
    print(f"Saved: {png_path}")

if __name__ == '__main__':
    render_svg(
        '/home/lxy/asr_sdm_robo/docs/article/IEEE-JSEN-LaTeX-template-202405/spa-klt-graphical-abstract-v3.svg',
        '/home/lxy/asr_sdm_robo/docs/article/IEEE-JSEN-LaTeX-template-202405/spa-klt-graphical-abstract-v3.png'
    )
