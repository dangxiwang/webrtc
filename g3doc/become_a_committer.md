# How to become WebRTC committer

<?% config.freshness.owner = 'titovartem' %?>
<?% config.freshness.reviewed = '2021-07-05' %?>

## WebRTC committer duties

WebRTC committers are responsible for keeping WebRTC codebase in a good shape
including, but not limited to the following aspects:

*   Code complexity and correctness
*   C++ best practices
*   Code formatting
*   Test coverage
*   Class/function level and conceptual documentation

Whenever a committer sets `Code Review +1` label on the CL, they approve that
the CL fulfills WebRTC style guides, language mastery, testability and
documentation. Being a committer means being responsible for WebRTC codebase
health and code quality.

## Becoming a WebRTC committer

To write code in WebRTC you don't need to be a committer, but to submit code to
WebRTC you do. So if you don't plan to work on the WebRTC codebase regularly,
you can ask other committers through code review to submit your patches, but if
you are going to work in the WebRTC codebase, then it's recommended to apply for
WebRTC committer rights obtaining process.

### If you don't write in C++

For non-C++ code authors please reachout to **PUT EMAIL HERE** to request WebRTC
committers rights.

### If you are going to write in C++

1.  Make yourself familiar with with C++ style guides:

    *   [Chromium style guide](https://chromium.googlesource.com/chromium/src/+/refs/heads/main/styleguide/c++/c++.md)
    *   [WebRTC style guide](https://webrtc.googlesource.com/src/+/refs/heads/main/g3doc/style-guide.md)

2.  Create a ticket to obtain WebRTC committers rights in Monorail.

    TODO: add link to the template.

3.  Pick a mentor among WebRTC committers, who will review your CLs
    demonstrating C++ readability skills. It's recomended to ask someone who is
    familiar with code base which you will be working on (you can chech OWNERS
    files to find such person). Otherwise you can reach out to committers
    mailing list committers@webrtc.org.

4.  Send CLs demonstrating C++ readability skills to the mentor for review and
    attach them to the created ticket.

5.  When the mentor decides that C++ readability skills are good enough they
    will send a proposal for granting WebRTC committer rights to the reviewing
    committee mailing list to review. If the proposal will be approved, then
    committer rights will be granted. Committee members will have up to 5
    business days to answer. In case of rejection detailed feedback on what
    aspects should be improved will be provided.
