#!/usr/bin/env python3
"""
SVG -> PNG renderer for spa-klt-graphical-abstract-v2.svg
Parses basic SVG elements and renders to PNG using cairo.
"""
import cairo
import math
from lxml import etree
from PIL import Image

NS = "http://www.w3.org/2000/svg"

def hex_to_rgb(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i+2], 16)/255.0 for i in (0, 2, 4))

def parse_color(c, default=(0,0,0)):
    if not c or c == 'none':
        return None
    if c.startswith('#'):
        return hex_to_rgb(c)
    if c.startswith('rgb'):
        # rgb(r,g,b) or rgba(r,g,b,a)
        inner = c[c.index('(')+1:c.index(')')]
        parts = [x.strip().strip('%') for x in inner.split(',')]
        r = float(parts[0])/255 if '%' in c else float(parts[0])/255
        g = float(parts[1])/255 if len(parts) > 1 else 0
        b = float(parts[2])/255 if len(parts) > 2 else 0
        return (r, g, b)
    return default

def parse_float(s, default=0.0):
    try:
        return float(s)
    except:
        return default

def parse_dasharray(s):
    if not s:
        return []
    return [float(x) for x in s.split()]

class SVGRenderer:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, int(width), int(height))
        self.ctx = cairo.Context(self.surface)
        self.ctx.set_source_rgb(1, 1, 1)
        self.ctx.paint()
        self.font_size_cache = {}

    def render(self, root):
        # Process all elements in document order
        self._render_element(root)

    def _render_element(self, elem):
        tag = elem.tag.split('}')[-1] if '}' in elem.tag else elem.tag

        if tag == 'rect':
            self._draw_rect(elem)
        elif tag == 'text':
            self._draw_text(elem)
        elif tag == 'line':
            self._draw_line(elem)
        elif tag == 'circle':
            self._draw_circle(elem)
        elif tag == 'polygon':
            self._draw_polygon(elem)
        elif tag == 'path':
            self._draw_path(elem)
        elif tag == 'defs':
            pass  # defs processed separately
        elif tag == 'g':
            self._render_group(elem)

    def _render_group(self, g):
        # Save state
        self.ctx.save()

        # Apply group transform if any
        transform = g.get('transform', '')
        if transform:
            self._apply_transform(transform)

        # Apply opacity to group
        opacity = parse_float(g.get('opacity', '1'), 1.0)
        if opacity < 1.0:
            self.ctx.push_group()
            for child in g:
                self._render_element(child)
            self.ctx.pop_group_to_source()
            self.ctx.paint_with_alpha(opacity)
        else:
            for child in g:
                self._render_element(child)

        self.ctx.restore()

    def _apply_transform(self, transform):
        # Parse translate, scale, rotate, matrix
        parts = transform.split(')')
        for part in parts:
            part = part.strip() + ')'
            if part == ')':
                continue
            if 'translate' in part:
                numbers = part.split('(')[-1].strip(')').split()
                tx = float(numbers[0])
                ty = float(numbers[1]) if len(numbers) > 1 else 0
                self.ctx.translate(tx, ty)
            elif 'scale' in part:
                numbers = part.split('(')[-1].strip(')').split()
                sx = float(numbers[0])
                sy = float(numbers[1]) if len(numbers) > 1 else sx
                self.ctx.scale(sx, sy)
            elif 'rotate' in part:
                numbers = part.split('(')[-1].strip(')').split()
                angle = math.radians(float(numbers[0]))
                cx = float(numbers[1]) if len(numbers) > 1 else 0
                cy = float(numbers[2]) if len(numbers) > 2 else 0
                if cx or cy:
                    self.ctx.translate(cx, cy)
                    self.ctx.rotate(angle)
                    self.ctx.translate(-cx, -cy)
                else:
                    self.ctx.rotate(angle)
            elif 'matrix' in part:
                numbers = part.split('(')[-1].strip(')').split()
                m = [float(x) for x in numbers]
                self.ctx.transform(cairo.Matrix(m[0], m[1], m[2], m[3], m[4], m[5]))

    def _draw_rect(self, rect):
        x = parse_float(rect.get('x', '0'))
        y = parse_float(rect.get('y', '0'))
        w = parse_float(rect.get('width', '0'))
        h = parse_float(rect.get('height', '0'))
        rx = parse_float(rect.get('rx', '0'), 0.0)
        ry = parse_float(rect.get('ry', None), rx)
        fill = rect.get('fill', 'none')
        stroke = rect.get('stroke', 'none')
        sw = parse_float(rect.get('stroke-width', '1'))
        opacity = parse_float(rect.get('opacity', '1'), 1.0)

        if opacity < 1.0:
            self.ctx.push_group()
            self._do_rect(x, y, w, h, rx, ry, fill, stroke, sw)
            self.ctx.pop_group_to_source()
            self.ctx.paint_with_alpha(opacity)
        else:
            self._do_rect(x, y, w, h, rx, ry, fill, stroke, sw)

    def _do_rect(self, x, y, w, h, rx, ry, fill, stroke, sw):
        if rx == 0 and ry == 0:
            self.ctx.rectangle(x, y, w, h)
        else:
            # Rounded rectangle using arc
            r = min(rx, ry, w/2, h/2)
            self.ctx.move_to(x + r, y)
            self.ctx.line_to(x + w - r, y)
            self.ctx.arc(x + w - r, y + r, r, -math.pi/2, 0)
            self.ctx.line_to(x + w, y + h - r)
            self.ctx.arc(x + w - r, y + h - r, r, 0, math.pi/2)
            self.ctx.line_to(x + r, y + h)
            self.ctx.arc(x + r, y + h - r, r, math.pi/2, math.pi)
            self.ctx.line_to(x, y + r)
            self.ctx.arc(x + r, y + r, r, math.pi, -math.pi/2)
            self.ctx.close_path()

        if fill and fill != 'none':
            c = parse_color(fill)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.fill_preserve()
            else:
                self.ctx.set_source_rgba(0,0,0,0)
                self.ctx.fill_preserve()

        if stroke and stroke != 'none':
            c = parse_color(stroke)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.set_line_width(sw)
                self.ctx.stroke()
        elif fill == 'none' or not fill:
            self.ctx.new_path()

    def _draw_text(self, text_elem):
        x = parse_float(text_elem.get('x', '0'))
        y = parse_float(text_elem.get('y', '0'))
        anchor = text_elem.get('text-anchor', 'start')
        fill_color = parse_color(text_elem.get('fill', '#000000'), (0,0,0))
        font_size = parse_float(text_elem.get('font-size', '12'))
        font_weight = text_elem.get('font-weight', 'normal')
        font_style = text_elem.get('font-style', 'normal')
        font_family = text_elem.get('font-family', 'Arial, Helvetica, sans-serif')

        # Parse font weight
        fw = cairo.FONT_WEIGHT_NORMAL
        if font_weight in ('bold', '700', '800', '900'):
            fw = cairo.FONT_WEIGHT_BOLD

        fs = cairo.FONT_SLANT_NORMAL
        if font_style == 'italic':
            fs = cairo.FONT_SLANT_ITALIC

        self.ctx.select_font_face(font_family.split(',')[0].strip(), fs, fw)
        self.ctx.set_font_size(font_size)

        if fill_color:
            self.ctx.set_source_rgb(*fill_color)

        # Handle tspan children
        if len(text_elem):
            # Multi-line text
            dy = 0
            for child in text_elem:
                ctag = child.tag.split('}')[-1] if '}' in child.tag else child.tag
                if ctag == 'tspan':
                    child_dy = parse_float(child.get('dy', '0'))
                    y_pos = y + dy + child_dy
                    self._draw_tspan(child, x, y_pos, anchor, font_family, font_size, fw, fs, fill_color)
                    # Estimate line height
                    dy += font_size * 1.25
        else:
            # Single line text
            content = (text_elem.text or '').strip()
            if not content:
                return
            self._render_single_text(content, x, y, anchor, font_size, fw, fs)

    def _draw_tspan(self, tspan, x, y, anchor, font_family, font_size, fw, fs, fill_color):
        content = (tspan.text or '').strip()
        if not content:
            return
        self._render_single_text(content, x, y, anchor, font_size, fw, fs)

    def _render_single_text(self, text, x, y, anchor, font_size, fw, fs):
        self.ctx.select_font_face('Arial', fs, fw)
        self.ctx.set_font_size(font_size)
        extents = self.ctx.text_extents(text)
        if anchor == 'middle':
            x -= extents.width / 2 + extents.x_bearing
        elif anchor == 'end':
            x -= extents.width + extents.x_bearing
        self.ctx.move_to(x, y)
        self.ctx.show_text(text)
        self.ctx.new_path()

    def _draw_line(self, line):
        x1 = parse_float(line.get('x1', '0'))
        y1 = parse_float(line.get('y1', '0'))
        x2 = parse_float(line.get('x2', '0'))
        y2 = parse_float(line.get('y2', '0'))
        stroke = line.get('stroke', '#000000')
        sw = parse_float(line.get('stroke-width', '1'))
        dash = parse_dasharray(line.get('stroke-dasharray', ''))

        c = parse_color(stroke)
        if not c:
            return
        self.ctx.set_source_rgb(*c)
        self.ctx.set_line_width(sw)
        if dash:
            self.ctx.set_dash(dash)
        self.ctx.move_to(x1, y1)
        self.ctx.line_to(x2, y2)
        self.ctx.stroke()
        self.ctx.set_dash([])
        self.ctx.new_path()

    def _draw_circle(self, circle):
        cx = parse_float(circle.get('cx', '0'))
        cy = parse_float(circle.get('cy', '0'))
        r = parse_float(circle.get('r', '0'))
        fill = circle.get('fill', 'none')
        stroke = circle.get('stroke', 'none')
        sw = parse_float(circle.get('stroke-width', '1'))

        self.ctx.arc(cx, cy, r, 0, 2*math.pi)
        if fill and fill != 'none':
            c = parse_color(fill)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.fill_preserve()
        if stroke and stroke != 'none':
            c = parse_color(stroke)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.set_line_width(sw)
                self.ctx.stroke()
        self.ctx.new_path()

    def _draw_polygon(self, polygon):
        points = polygon.get('points', '')
        fill = polygon.get('fill', 'none')
        stroke = polygon.get('stroke', 'none')
        sw = parse_float(polygon.get('stroke-width', '1'))

        pts = [float(x) for x in points.replace(',', ' ').split()]
        if len(pts) < 4:
            return
        self.ctx.move_to(pts[0], pts[1])
        for i in range(2, len(pts), 2):
            self.ctx.line_to(pts[i], pts[i+1])
        self.ctx.close_path()

        if fill and fill != 'none':
            c = parse_color(fill)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.fill_preserve()
        if stroke and stroke != 'none':
            c = parse_color(stroke)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.set_line_width(sw)
                self.ctx.stroke()
        self.ctx.new_path()

    def _draw_path(self, path):
        d = path.get('d', '')
        fill = path.get('fill', 'none')
        stroke = path.get('stroke', 'none')
        sw = parse_float(path.get('stroke-width', '1'))
        fill_rule = path.get('fill-rule', 'nonzero')

        self._path_from_d(d)
        if fill_rule == 'evenodd':
            self.ctx.set_fill_mode(cairo.FILL_RULE_EVEN_ODD)

        if fill and fill != 'none':
            c = parse_color(fill)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.fill_preserve()
        if stroke and stroke != 'none':
            c = parse_color(stroke)
            if c:
                self.ctx.set_source_rgb(*c)
                self.ctx.set_line_width(sw)
                self.ctx.stroke()
        self.ctx.new_path()
        self.ctx.set_fill_mode(cairo.FILL_RULE_WINDING)

    def _path_from_d(self, d):
        """Parse SVG path d attribute and draw with cairo."""
        import re
        # Tokenize
        tokens = re.findall(r'[MmZzLlHhVvCcSsQqTtAa]|-?\d+\.?\d*(?:[eE][-+]?\d+)?', d)
        i = 0
        current_x, current_y = 0, 0
        start_x, start_y = 0, 0

        def get_num():
            nonlocal i
            if i >= len(tokens):
                return None
            t = tokens[i]
            i += 1
            try:
                return float(t)
            except:
                return None

        def cmd():
            nonlocal i
            if i >= len(tokens):
                return None
            t = tokens[i]
            i += 1
            return t

        while i < len(tokens):
            op = tokens[i] if not tokens[i].isalpha() else None
            if op:
                i += 1

            if i == 0 or (i < len(tokens) and not tokens[i].isalpha()):
                pass
            else:
                if i < len(tokens) and tokens[i].isalpha():
                    op = tokens[i]; i += 1

            # Read command
            if i == 0 and i < len(tokens):
                c = tokens[i]; i += 1
            else:
                c = op if op else 'L'

            if c == 'M':
                x = get_num(); y = get_num()
                if x is None: break
                current_x = x; current_y = y
                start_x = x; start_y = y
                self.ctx.move_to(x, y)
                # Implicit lineto
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    x = float(t); y = get_num()
                    self.ctx.line_to(x, y)
                    current_x = x; current_y = y
            elif c == 'm':
                x = get_num(); y = get_num()
                if x is None: break
                x += current_x; y += current_y
                current_x = x; current_y = y
                start_x = x; start_y = y
                self.ctx.move_to(x, y)
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    dx = float(t); dy = get_num()
                    x = current_x + dx; y = current_y + dy
                    self.ctx.line_to(x, y)
                    current_x = x; current_y = y
            elif c == 'L':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    x = float(t); y = get_num()
                    self.ctx.line_to(x, y)
                    current_x = x; current_y = y
            elif c == 'l':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    dx = float(t); dy = get_num()
                    x = current_x + dx; y = current_y + dy
                    self.ctx.line_to(x, y)
                    current_x = x; current_y = y
            elif c == 'H':
                x = get_num()
                if x is None: break
                self.ctx.line_to(x, current_y)
                current_x = x
            elif c == 'h':
                dx = get_num()
                if dx is None: break
                x = current_x + dx
                self.ctx.line_to(x, current_y)
                current_x = x
            elif c == 'V':
                y = get_num()
                if y is None: break
                self.ctx.line_to(current_x, y)
                current_y = y
            elif c == 'v':
                dy = get_num()
                if dy is None: break
                y = current_y + dy
                self.ctx.line_to(current_x, y)
                current_y = y
            elif c == 'Z' or c == 'z':
                self.ctx.close_path()
                current_x = start_x; current_y = start_y
            elif c == 'C':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    x1 = float(t); y1 = get_num()
                    x2 = float(tokens[i]); y2 = get_num()
                    x = float(tokens[i]); y = get_num()
                    self.ctx.curve_to(x1, y1, x2, y2, x, y)
                    current_x = x; current_y = y
            elif c == 'c':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    dx1 = float(t); dy1 = get_num()
                    dx2 = float(tokens[i]); dy2 = get_num()
                    dx = float(tokens[i]); dy = get_num()
                    x1 = current_x + dx1; y1 = current_y + dy1
                    x2 = current_x + dx2; y2 = current_y + dy2
                    x = current_x + dx; y = current_y + dy
                    self.ctx.curve_to(x1, y1, x2, y2, x, y)
                    current_x = x; current_y = y
            elif c == 'S':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    x2 = float(t); y2 = get_num()
                    x = float(tokens[i]); y = get_num()
                    self.ctx.curve_to(current_x, current_y, x2, y2, x, y)
                    current_x = x; current_y = y
            elif c == 's':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    dx2 = float(t); dy2 = get_num()
                    dx = float(tokens[i]); dy = get_num()
                    x2 = current_x + dx2; y2 = current_y + dy2
                    x = current_x + dx; y = current_y + dy
                    self.ctx.curve_to(current_x, current_y, x2, y2, x, y)
                    current_x = x; current_y = y
            elif c == 'A' or c == 'a':
                while i < len(tokens):
                    t = tokens[i]
                    if t.isalpha(): break
                    rx = float(t); ry = get_num()
                    x_rot = get_num()
                    large = get_num()
                    sweep = get_num()
                    dx = float(tokens[i]); dy = get_num()
                    if c == 'a':
                        dx = current_x + dx
                        dy = current_y + dy
                    self.ctx.line_to(dx, dy)
                    current_x = dx; current_y = dy
            else:
                break

    def save(self, path):
        # Convert BGRA to RGBA before saving
        buf = self.surface.get_data()
        import numpy as np
        arr = np.frombuffer(buf, dtype=np.uint8).copy()
        h, w = int(self.height), int(self.width)
        arr = arr.reshape(h, w, 4)
        # BGRA -> RGBA
        arr = arr[:, :, [2, 1, 0, 3]]
        from PIL import Image
        img = Image.fromarray(arr, 'RGBA').convert('RGB')
        img.save(path, 'PNG')


def main():
    svg_path = '/home/lxy/asr_sdm_robo/docs/article/IEEE-JSEN-LaTeX-template-202405/spa-klt-graphical-abstract-v2.svg'
    png_path = '/home/lxy/asr_sdm_robo/docs/article/IEEE-JSEN-LaTeX-template-202405/spa-klt-graphical-abstract-v2.png'

    tree = etree.parse(svg_path)
    root = tree.getroot()

    # Get viewBox or width/height
    vb = root.get('viewBox', '').split()
    if len(vb) == 4:
        vx, vy, vw, vh = float(vb[0]), float(vb[1]), float(vb[2]), float(vb[3])
    else:
        vw = float(root.get('width', 1920))
        vh = float(root.get('height', 1080))

    print(f"Rendering SVG {int(vw)}x{int(vh)}...")

    r = SVGRenderer(vw, vh)
    r.render(root)

    # Save PNG
    r.save(png_path)
    print(f"Saved: {png_path}")

    # Also convert to RGB for display
    pil_img = r.to_pil()
    rgb = pil_img.convert('RGB')
    rgb.save(png_path.replace('.png', '_rgba.png'))
    print(f"RGB PNG also saved")


if __name__ == '__main__':
    main()
