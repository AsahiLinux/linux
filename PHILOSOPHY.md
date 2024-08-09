# `accurate` branch
Being a pure, plain decompilation of the original `Doukutsu.exe` file (v1.0.0.6),
there should not be much to say about this branch's philosophies:

## Goal
The end-goal is for the project to be able to produce an `.exe` file that is
identical to the original. This means that there should be no custom code,
decompiled code should ideally be made to produce the same assembly code as the
original, bugs should be left intact, etc.

Another goal of the project is to document Cave Story's inner-working, so code
should be reasonably-annotated. Likewise, bugs should be documented, and fixes
provided wrapped in `#ifdef FIX_BUGS` conditions.

## Accuracy to the original source code
Personally, I do aim to make the decompiled code _functionally_-accurate to the
original, down to generating the same assembly code, but I do not aim for
_visually_-accurate code.

Despite this, I do try to preserve the original function/variable names,
variable-declaration locations, and source file naming.

Part of the reason why I do not aim for visually-accurate source code is that we
know from the [Organya source code release](https://github.com/shbow/organya/)
what Pixel's code-style looked like, and I find it **extremely** hard to read.

## Language
Cave Story's developer (Pixel) primarily speaks Japanese, but his code's
variable/function names are mostly written in English (with a few rare instances
of Romanised Japanese).

The Organya source code release indicates that Pixel wrote his comments in
Japanese, however, in this project, I prefer them to be written in English.

The English employed in this project is a mix of American English, Canadian
English, and British English.



# `portable` branch
This branch takes a different direction to the `accurate` branch, but they still
share many core philosophies:

## Goal
While accurately-reproducing Cave Story's original code is still a priority, the
main focus of this branch is to port the game to other platforms, while also
preserving the game the way it was experienced back in 2004.

Essentially, this branch exists to provide a purist Cave Story experience
without requiring an old Windows XP computer.

Notably, this means that bugs should still be left intact. However, bugs and
other coding errors that affect portability should be fixed.

For comparison, I believe this branch shares many parallels with the
Chocolate Doom project. Follow the link below to see that project's list of
philosophies, which may be applicable here:

https://github.com/chocolate-doom/chocolate-doom/blob/master/PHILOSOPHY.md

## Custom code
Cave Story's original source code was written in C++, but stylised like C89
(with a number of exceptions). Custom code added to the project should follow
suit, but the C-style doesn't have to be C89 - personally, I prefer to write in
the subset of C99 that C++98 allows.

I prefer compiler-specific code to be avoided, since more-portable code benefits
all compilers, and keeps the codebase free of clutter.
