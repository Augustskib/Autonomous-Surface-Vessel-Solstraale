/* Sequenced operator controls. The arbiter, not the button, confirms authority. */
class OperatorControls {
    constructor({post, notice, changed, session}) {
        this.post = post; this.notice = notice; this.changed = changed;
        this.session = session; this.seq = 0; this.revision = 0;
        this.intent = 'HOLD'; this.confirmed = false; this.pending = false;
        this.left = 0; this.right = 0; this.ack = null;
    }
    async send(transition) {
        const revision = this.revision;
        const seq = ++this.seq;
        this.pending = true;
        try {
            const result = await this.post('/operator', {
                session: this.session, seq, intent: this.intent, transition,
                left: this.left, right: this.right,
            });
            if (revision !== this.revision) return false;
            const ack = result.operator;
            if (!ack || ack.request_session !== this.session || ack.request_seq !== seq)
                throw new Error('Missing matching control acknowledgement');
            if (!ack.accepted) throw new Error(ack.reason || 'Control rejected');
            this.ack = ack; this.confirmed = true;
            this.changed(this);
            return true;
        } catch (error) {
            if (revision !== this.revision) return false;
            this.intent = 'HOLD'; this.left = this.right = 0;
            this.confirmed = false; this.pending = false;
            this.notice(error.message + ' Choose Take Manual or Continue Mission to retry.');
            this.changed(this);
            // Best-effort explicit stop. If the link is down, the arbiter's
            // one-second watchdog also latches Hold, including during AUTO.
            ++this.revision;
            this.post('/operator', {session:this.session, seq:++this.seq,
                intent:'HOLD', transition:true, left:0, right:0}).catch(() => {});
            return false;
        } finally {
            if (revision === this.revision) this.pending = false;
        }
    }
    transition(intent) {
        ++this.revision;
        this.intent = intent; this.left = this.right = 0;
        this.confirmed = false; this.pending = false;
        this.changed(this);
        return this.send(true);
    }
    heartbeat() {
        if (this.confirmed && !this.pending && ['MANUAL','AUTO'].includes(this.intent))
            return this.send(false);
        return Promise.resolve(false);
    }
    observe(status, age) {
        if (!this.confirmed || this.pending || !['MANUAL','AUTO'].includes(this.intent)) return;
        if (age == null || age > 1 || !status || status.session !== this.session
                || status.expired || status.mode === 'ESTOP') {
            ++this.revision;
            this.intent = 'HOLD'; this.left = this.right = 0;
            this.confirmed = false; this.pending = false;
            this.notice('Control is no longer confirmed. Choose Take Manual or Continue Mission explicitly.');
            this.changed(this);
        }
    }
}

if (typeof module !== 'undefined') module.exports = {OperatorControls};
if (typeof document !== 'undefined') {
    // All errors stay in the page. A native alert would freeze drive timers.
    function notifyOperator(message) {
        const el = document.getElementById('operator_notice');
        if (el) el.textContent = message;
    }
    async function postCommand(url, body) {
        const abort = new AbortController();
        const timer = setTimeout(() => abort.abort(), 2200);
        try {
            const response = await fetch(url, {method:'POST',
                headers:{'Content-Type':'application/json'},
                body:JSON.stringify(body), signal:abort.signal});
            const data = await response.json();
            if (!response.ok || data.status !== 'ok')
                throw new Error(data.error || 'Command was not accepted');
            return data;
        } catch (error) {
            if (error.name === 'AbortError') throw new Error('Control request timed out');
            throw error;
        } finally { clearTimeout(timer); }
    }
    const operator = new OperatorControls({post:postCommand, notice:notifyOperator,
        session:Array.from(crypto.getRandomValues(new Uint8Array(16)),
            b => b.toString(16).padStart(2,'0')).join(''), changed: state => {
            if (['HOLD','ESTOP','RESET'].includes(state.intent)) clearDriveKeys();
            const el = document.getElementById('lease_state');
            if (el) el.textContent = state.confirmed
                ? `${state.ack.mode} confirmed` : `${state.intent}: not confirmed`;
        }});
    function clearDriveKeys() {
        for (const key of Object.keys(keys)) keys[key] = false;
        operator.left = operator.right = left = right = 0;
        setBar('left_bar', 0); setBar('right_bar', 0);
        document.getElementById('left_val').innerText = 0;
        document.getElementById('right_val').innerText = 0;
    }
    function acquireLease() {
        if (operator.intent === 'MANUAL' && (operator.confirmed || operator.pending))
            return Promise.resolve(operator.confirmed);
        notifyOperator('Taking manual control; the mission is retained.');
        return operator.transition('MANUAL');
    }
    function releaseLease() {
        clearDriveKeys();
        notifyOperator('Holding zero thrust. The remaining route is retained.');
        return operator.transition('HOLD');
    }
    async function resumeAuto() {
        clearDriveKeys();
        if (!missionActive) {
            notifyOperator('No active mission to continue. Plot a route and choose Start Mission.');
            return false;
        }
        // A reloaded page has a new session. Explicitly claim Hold first.
        if (!operator.ack && !await operator.transition('HOLD')) return false;
        notifyOperator('Continuing the retained mission.');
        return operator.transition('AUTO');
    }
    function updateThrust() {
        if (operator.intent === 'MANUAL') {
            let l=0, r=0;
            if (keys.w) { l+=MANUAL_THRUST_LIMIT; r+=MANUAL_THRUST_LIMIT; }
            if (keys.s) { l-=MANUAL_THRUST_LIMIT; r-=MANUAL_THRUST_LIMIT; }
            if (keys.a) { l-=MANUAL_THRUST_LIMIT; r+=MANUAL_THRUST_LIMIT; }
            if (keys.d) { l+=MANUAL_THRUST_LIMIT; r-=MANUAL_THRUST_LIMIT; }
            operator.left = left = Math.max(-MANUAL_THRUST_LIMIT, Math.min(MANUAL_THRUST_LIMIT,l));
            operator.right = right = Math.max(-MANUAL_THRUST_LIMIT, Math.min(MANUAL_THRUST_LIMIT,r));
            document.getElementById('left_val').innerText = left;
            document.getElementById('right_val').innerText = right;
            setBar('left_bar',left);setBar('right_bar',right);
        }
        operator.heartbeat();
    }
    setInterval(updateThrust, 200);
    const DRIVE_KEYS = ['w','a','s','d'];
    document.addEventListener('keydown', e => {
        if (e.target.closest('input, textarea, select, [contenteditable=true]')) return;
        const k=e.key.toLowerCase();
        if (!DRIVE_KEYS.includes(k)) return;
        e.preventDefault();
        // Ignore autorepeat after a control failure. A new keypress is explicit.
        if (e.repeat && operator.intent !== 'MANUAL') return;
        keys[k]=true;acquireLease();
    });
    document.addEventListener('keyup',e=>{keys[e.key.toLowerCase()]=false;});
    function holdOnBlur() {
        if (['MANUAL','AUTO'].includes(operator.intent)) releaseLease();
    }
    window.addEventListener('blur',holdOnBlur);
    document.addEventListener('visibilitychange',()=>{if(document.hidden)holdOnBlur();});
    window.addEventListener('beforeunload',()=>{
        if (!operator.ack) return; // Closing a spectator tab cannot seize control.
        navigator.sendBeacon('/operator',new Blob([JSON.stringify({
            session:operator.session,seq:++operator.seq,intent:'HOLD',
            transition:true,left:0,right:0})],{type:'application/json'}));
    });
    // Never shadow native alert globally; these helpers are only for this page.
    Object.assign(window,{notifyOperator,postCommand,operator,clearDriveKeys,
        acquireLease,releaseLease,resumeAuto,updateThrust});
}
