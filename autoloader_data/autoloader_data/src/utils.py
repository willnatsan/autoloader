import numpy


def flatten(arr: list[list[int]] | list[list[float]]) -> list[int] | list[float]:
    return numpy.array(arr).flatten().tolist()


def reshape(
    arr: list[int] | list[float], width: int
) -> list[list[int]] | list[list[float]]:
    return numpy.reshape(arr, (-1, width)).tolist()
