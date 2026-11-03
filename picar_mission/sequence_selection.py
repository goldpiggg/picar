"""Read-only selection of one of the four agreed diagonal PICK plans."""
from dataclasses import dataclass
import math
from .qr_scan import normalize

START_POSITIONS = ('LEFT-A', 'LEFT-B', 'RIGHT-A', 'RIGHT-B')


def sequence_for(start_position):
    if start_position not in START_POSITIONS:
        raise ValueError('Unknown sequence start position')
    side, row = start_position.split('-')
    other = 'RIGHT' if side == 'LEFT' else 'LEFT'
    diagonal = 'B' if row == 'A' else 'A'
    return dict(number=START_POSITIONS.index(start_position)+1,
        start_position=start_position,
        steps=[dict(position=position, action=action) for position,action in (
            (start_position,'PICK'), (other+'-'+row,'PLACE'),
            (other+'-'+diagonal,'PICK'), (side+'-'+diagonal,'PLACE'))])


# A lone code must sit at least this far from the image center to count.
SINGLE_CODE_MARGIN = .15


def classify_side_start(detections, direction, min_gap=.05, settled=False):
    """Compare the two QR centers after turning toward a single side.

    Confirmed camera view: RIGHT's A is screen-left, LEFT's A screen-right.
    While the arm turns, image halves and vertical coordinates do not
    identify physical sides, so both codes are required. Once the turn has
    finished (`settled`), the two spots straddle the image center, and one
    code clearly on one side is enough: a crumpled sheet often hides the
    other one from the decoder.
    """
    if direction not in ('LEFT', 'RIGHT'):
        return None
    codes = {}
    for item in detections:
        label = normalize(item.get('text', ''))
        if label not in ('PICK', 'PLACE'):
            continue
        if label in codes:
            return None
        try:
            x = item['center_normalized'][0]
        except (KeyError, IndexError, TypeError):
            return None
        if type(x) not in (int, float) or not math.isfinite(x) or not 0 <= x <= 1:
            return None
        codes[label] = x
    if set(codes) == {'PICK', 'PLACE'}:
        if abs(codes['PICK']-codes['PLACE']) < min_gap:
            return None
        pick_right = codes['PICK'] > codes['PLACE']
    elif settled and len(codes) == 1:
        (label, x), = codes.items()
        if abs(x-.5) < SINGLE_CODE_MARGIN:
            return None
        pick_right = (x > .5) == (label == 'PICK')
    else:
        return None
    row = 'A' if pick_right == (direction == 'LEFT') else 'B'
    return direction+'-'+row


@dataclass
class SideQrGate:
    required_frames: int = 3
    min_gap: float = .05
    max_gap_seconds: float = 1.5
    candidate: str = None
    count: int = 0
    last_seen: float = None
    confirmed: str = None

    def update(self, detections, direction, now, settled=False):
        if self.confirmed is not None:
            return self.confirmed
        start = classify_side_start(detections, direction, self.min_gap, settled)
        gap = self.last_seen is not None and now-self.last_seen > self.max_gap_seconds
        if start is None or start != self.candidate or gap:
            self.candidate, self.count = start, int(start is not None)
        else:
            self.count += 1
        self.last_seen = now if start is not None else None
        if start is not None and self.count >= self.required_frames:
            self.confirmed = start
            return start
        return None
