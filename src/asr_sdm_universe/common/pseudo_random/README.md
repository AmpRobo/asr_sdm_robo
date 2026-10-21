# rng.c

C library for generating pseudo random numbers. Using the principles from
[pcg-random.org](https://pcg-random.org).

## Example usage

Seeding (requires two ints):

```c
rng_seed(time(NULL), 1);
```

Generating a random 32-bit integer:

```c
uint32_t roll = rng_u();
```

## Blog article

Read more about how and why I created this library:

[Stop using `rand()`](https://vanderoost.com/articles/2026/09/25/stop-using-rand/)

## Screencasts

Walkthrough where I'm cooking up the code from scratch:

[![Coding RNG from scratch in C](https://img.youtube.com/vi/-5NqTA-fDNQ/mqdefault.jpg)](https://youtu.be/-5NqTA-fDNQ)

Improving the random float generator:

[![Generating better random floats in C](https://img.youtube.com/vi/qa83_KrkVIM/mqdefault.jpg)](https://youtu.be/qa83_KrkVIM)
