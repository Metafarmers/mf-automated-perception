import seaborn

distinct_rgbs = seaborn.color_palette("husl", 20)
def index_to_distinct_rgb(index, max255=True):
  output = distinct_rgbs[index % len(distinct_rgbs)]
  if max255:
    out = [int(c * 255) for c in output]
    return tuple(out)
  return output
