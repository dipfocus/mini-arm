import logging
import os


LOG_FORMAT = "%(asctime)s | %(levelname)s | %(filename)s:%(lineno)d | %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def _parse_log_level(level: int | str) -> int:
    if isinstance(level, int):
        return level

    normalized = level.strip().upper()
    if not normalized:
        raise ValueError("log level cannot be empty")

    if normalized.lstrip("+-").isdigit():
        return int(normalized)

    candidate = getattr(logging, normalized, None)
    if isinstance(candidate, int):
        return candidate

    raise ValueError(f"invalid log level: {level!r}")


def configure_logging(level: int | str | None = None) -> None:
    effective_level = level if level is not None else os.getenv("LOG_LEVEL", "INFO")
    formatter = logging.Formatter(LOG_FORMAT, DATE_FORMAT)
    root_logger = logging.getLogger()
    root_logger.setLevel(_parse_log_level(effective_level))

    if not root_logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        root_logger.addHandler(handler)
        return

    for handler in root_logger.handlers:
        handler.setFormatter(formatter)
