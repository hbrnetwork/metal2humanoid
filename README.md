# Metal2Humanoid

**The [NAND2Tetris](https://www.nand2tetris.org/) of robotics.**  
A full-stack, open-source curriculum for building humanoid robots from first principles — starting with motors and ending with embodied intelligence.

Metal2Humanoid is a constructive journey through robotics. Instead of treating robots as black boxes, we build them layer by layer: metal → motion → behavior → learning → meaning. Each component is designed, simulated, tested, and understood in context.

This repository contains the **core materials for the Metal2Humanoid course**, including open-source robot embodiments, hands-on projects, and supporting documentation.

This course was made in collaboration with **[Menlo Research](https://news.asimov.inc/)**

---

## Course Philosophy

Robotics today is where computing was in the 1970s: powerful, expensive, fragmented, and opaque.

Metal2Humanoid is inspired by **[NAND2Tetris](https://www.nand2tetris.org/)** and the **Homebrew Computer Club** ethos:

- Start from primitives, not APIs  
- Build complete systems, not isolated demos  
- Prefer open, reproducible designs  
- Treat embodiment, intelligence, and ethics as inseparable  

By the end of the course, students don’t just *use* robots — they understand how robots come to exist.

---

## Repository Structure

---

## `embodiments/`

Open-source **robot embodiments** used throughout the course.

This folder contains CAD files, URDFs, BOMs, firmware, and reference code for:
- Actuators and motor modules
- Limbs and joints
- Full-body humanoid or mobile-manipulation platforms

Each embodiment is designed to be:
- Low-cost
- Reproducible
- Modular
- Simulation-first, hardware-optional

These embodiments serve as the *physical substrate* for the projects.

---

## `projects/`

The **hands-on projects** that define the course.

Each project corresponds to a major conceptual step in the curriculum and may include:
- PDF handouts
- Jupyter notebooks
- Starter code
- Evaluation criteria

Projects progress roughly as:

1. Metal → Motor  
2. Motor → Actuation  
3. Actuation → Design  
4. Design → Motion  
5. Motion → Manipulation  
6. Manipulation → Behavior  
7. Behavior → Models  
8. Models → Learning
9. Learning → Ideas (Reflection & Manifesto) 
10. Ideas → Humanoid (Final Integration)

Most projects can be completed fully in simulation; a few are intended to touch real hardware.

---

## `docs/`

**Supporting material and readings.**

This folder includes:
- Course notes
- Background readings
- References to textbooks and papers
- Philosophical and ethical material
- Links to external tools and resources

---

## Simulation First, Hardware Optional

Most systems are developed and validated in simulation (e.g. MuJoCo, PyBullet, Isaac).  
Hardware builds are encouraged but not required.

The philosophy is:

> **If you can’t simulate it, you don’t understand it.  
> If you can’t build it, you don’t own it.**

---

## Who This Is For

- Students learning robotics for the first time  
- Researchers who want a systems-level refresher  
- Builders and hobbyists interested in humanoids  
- Educators looking for a modern robotics curriculum  
- Anyone who wants to understand robots *from the inside out*

A background in programming or robotics is helpful, but not required.

---

## Open & Evolving

This repository is intentionally incomplete.

Robotics is changing quickly, and Metal2Humanoid is meant to evolve with:
- New embodiments
- Better actuators
- Improved learning methods
- Community contributions

Pull requests, forks, and extensions are encouraged.

---

## License

Unless otherwise specified:
- Code is released under an open-source license (see individual folders)
- Hardware designs follow open hardware licenses
- Documentation is openly shareable

Specific licenses are documented per subfolder.

---

## A Final Note

> “Nothing is more important than seeing the sources of invention,  
> which are, in my opinion, more interesting than the inventions themselves.”  
> — Leibniz

Metal2Humanoid is about those sources.

Welcome.
