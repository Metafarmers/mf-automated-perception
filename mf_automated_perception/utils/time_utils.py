from datetime import datetime, timedelta, timezone

_KST = timezone(timedelta(hours=9))

def now() -> datetime:
  return datetime.now(_KST)