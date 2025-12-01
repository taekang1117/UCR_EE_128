int main(void)
{
    init_gpio();
    init_systick();

    // Start in COUNTDOWN right away
    currentState      = STATE_COUNTDOWN;
    seconds_left      = 9;
    countdown_ms      = 9000;
    countdown_active  = 1;
    displayFlag       = 1;

    red_off();
    green_on();

    while (1) {
        if (timeoutFlag) {
            timeoutFlag = 0;
            // When timer finishes, go back to locked display 9/red on
            currentState      = STATE_LOCKED;
            countdown_active  = 0;
            countdown_ms      = 0;
            seconds_left      = 9;
            displayFlag       = 1;
            red_on();
            green_off();
        }

        if (displayFlag) {
            displayFlag = 0;
            display_digit(seconds_left % 10);
        }
    }
}
