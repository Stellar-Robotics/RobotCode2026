# Welcome to the repository for Stellar Robotics' 2026 robot code!

If your visiting from a different FRC team, please feel free to use our code for a little R&D (Rob and Duplicate).  This is under a GPLv3 license, so keep that in mind.

If your part of our team and are looking to contribute, **please read though the Stances and Practices** and then contact the programming mentor for commit access.

## The Idea

We're aiming to do a couple things different this year in hopes to make it easier for our fresh programmers moving up from FLL.

1. We're hoping to utilize Yet Another Generic Swerve Library or (YAGSL) to hopefully abstract away some of the swerve drive complexities while still retaining a solid drive-train.

2. While we've heared from fellow teams that PathPlanner plus Choreo are really powerful together, we'll stick with just using PathPlanner this year (assuming they release a 2026 version) as it's generally more intuitive and straight forward for fresh learners.

## Stances and Practices

For the students who would like to start contributing to the code base, lets go over a couple base policies.  These also apply to mentors (pointing at myself).

Firstly, let's aim not to push known broken code.  This is different than say a subsystem that isn't feature complete.  What I mean is that we don't want to push changes that break core systems or a subsystem's ability to communicate with said core system.

Secondly, Lets try to get into the habit of committing and pushing our changes regularly.  In a multi-developer scenario, we've found that smaller commits are easier to work with when it comes to resolving conflicts.

Now for the big one, lets talk about AI.  We've noticed over the prep season this year that we have some new students utilizing LLMs like ChatGPT for coding tasks.  While we're happy to see a new tool in the programmer's toolbox, we believe there is a line between using AI to learn, and to generate solutions.  I can say that I myself am guilty of using AI to generate my solutions.  When it comes to approaching AI we believe:

1. Students should be able to comprehend and explain code they generate (or seek to learn from such).

2. The student should be the origin of a final solution (or goal).

An example of responsible use of AI, for us could be a student (or mentor) using an LLM to understand existing code, mathematical, or syntactical concepts.  An example of irresponsible use of AI, for us could be a student (or mentor) using an LLM to generate a state machine or path following algorithm that said coder does not understand.
