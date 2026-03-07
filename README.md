This repository contains the latest source codes for opennurbs and
opencascade, both of which are open source repositories that work
with NURBS B-Rep geometry.

BRL-CAD uses opennurbs internally for its brep primitive representation,
and for both conversion and processing considerations it would be
potentially useful to move data into and out of opencascade.  However,
that means developing a translation layer that can migrate data to
and from opennurbs and opencascade.

Please study the two APIs and data representations, focusing in particular
on the classic NURBS B-Rep surfaces, curves and topology used to represent
solid volumes, and devise a plan to allow us to migrate opennurbs data
into opencascade and vice versa.  Our focus is not exclusively on NURBS
B-Reps, but that is by far the most important component - if that does
not work there is no point in anything else.

Please create a TRANSLATE.md file with a plan to create a library that
can bridge between these to systems.
