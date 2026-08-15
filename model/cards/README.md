# Cards

Playing-card models, sharing the card mesh and face textures in [`assets`](assets).

<p float="left">
  <img src="house_of_cards.png" width="400">
</p>

| Model | Description |
| --- | --- |
| [`cards.xml`](cards.xml) | A deck of loose cards. |
| [`house_of_cards.xml`](house_of_cards.xml) | A 26-card, four-storey house. |

The house stands on friction alone, and stays standing: settled, it drifts by about a millimetre
over two minutes of simulated time. Cards are 0.7 mm thick, so the contacts are the awkward kind —
nearly parallel faces meeting at shallow angles, where a collider has the least information to work
with. Two details carry the model:

* **Collision cards are a thin box with a capsule-rounded perimeter.** Edge contacts between thin
  boxes are degenerate; capsules along all four edges are well-conditioned. The visual card is a
  separate, non-colliding mesh.
* **Every card carries a baked-in yaw of a few tenths of a degree, and a fraction of a millimetre
  of jitter along the ridges.** Exactly symmetric, perfectly aligned stacks are adversarial for a
  collider: the pristine arrangement is the one that falls over.

Frictional stacking of this kind requires `cone="elliptic"` and a large `impratio`. Adding
`noslip_iterations` cuts the residual creep further, at some cost in speed; see the comments in
the file.

## Changelog

* 05-08-2026: Added `house_of_cards.xml`.
