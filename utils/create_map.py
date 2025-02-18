from dataclasses import dataclass

width = 182
height = 232
# width = 4
# height = 2
max_val = 255
filename = "map.pgm"

@dataclass
class Coordinate:
    x: int
    y: int

@dataclass
class Line:
    color:   str
    start:   Coordinate
    end:     Coordinate

def main():
    ok = b'\xEF'
    ng = b'\x00'
    or_not = b'\xCD'

    with open(filename, "wb") as f:
        header = f"P5\n{width} {height}\n{max_val}\n"
        f.write(header.encode("ascii"))

        # data = b'\xEF' * (width*height)
        data = [b'\xEF' for i in range(width*height)]

        line_list = [
            Line(ng, Coordinate(0, 0), Coordinate(51, 0)),
            # Line(ng, Coordinate(0, 0), Coordinate(0, 232)),
            Line(ng, Coordinate(0, 0), Coordinate(0, 51)),
            Line(ng, Coordinate(0, 51), Coordinate(0, 61)),
            Line(ng, Coordinate(0, 91), Coordinate(0, 126)),
            Line(ng, Coordinate(0, 156), Coordinate(0, 191)),
            Line(ng, Coordinate(0, 221), Coordinate(0, 231)),
            Line(ng, Coordinate(51, 0), Coordinate(51, 50)),
            Line(ng, Coordinate(51, 50), Coordinate(181, 50)),
            Line(ng, Coordinate(181, 50), Coordinate(181, 231)),
            Line(ng, Coordinate(0, 231), Coordinate(181, 231)),
            *[Line(ng, Coordinate(x_, 91), Coordinate(x_, 91+90)) for x_ in range(61, 61+28+1)],
            *[Line(or_not, Coordinate(52, y_), Coordinate(181, y_)) for y_ in range(0, 49+1)]
        ]
        for line_ in line_list:
            draw_line(data, line_)

        data = b''.join(data)
        f.write(data)

def draw_line(data: list, line: Line):
    assert line.start.x==line.end.x or line.start.y==line.end.y, "Error: "

    if line.start.x==line.end.x:
        x_ = line.start.x
        for y_ in range(line.start.y, line.end.y+1):
            set_grid(data, line.color, x_, y_)
    else:
        y_ = line.start.y
        for x_ in range(line.start.x, line.end.x+1):
            set_grid(data, line.color, x_, y_)


def set_grid(data: list, color, x, y):
    index = y*width + x
    data[index] = color

if __name__ == "__main__":
    main()