from flask import Flask, render_template, request, redirect, url_for, flash, jsonify
from flask_login import LoginManager, UserMixin, login_user, logout_user, login_required, current_user
from werkzeug.security import generate_password_hash, check_password_hash
import pymysql
from ros2_bridge import ros2_bridge

app = Flask(__name__)
app.secret_key = 'totev-on-top'

# Database Configuration
def get_db():
    return pymysql.connect(host='127.0.0.1', user='root', password='', database='light_monitor', cursorclass=pymysql.cursors.DictCursor, autocommit=True)

class User(UserMixin):
    """Minimal user model for session management."""
    def __init__(self, id, username):
        self.id = id; self.username = username

login_manager = LoginManager()
login_manager.init_app(app)
login_manager.login_view = 'login'

@login_manager.user_loader
def load_user(uid):
    """Retrieve user context for the current session."""
    with get_db().cursor() as c:
        c.execute('SELECT * FROM users WHERE id=%s', (uid,))
        u = c.fetchone()
        return User(u['id'], u['username']) if u else None

@app.route('/')
def index(): return redirect(url_for('boot'))

@app.route('/boot')
def boot(): return redirect(url_for('desktop')) if current_user.is_authenticated else render_template('boot.html')

@app.route('/desktop')
@login_required
def desktop(): return render_template('desktop.html', data=ros2_bridge.get_data())

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        u = request.form.get('username')
        p = request.form.get('password')
        with get_db().cursor() as c:
            c.execute('SELECT * FROM users WHERE username=%s', (u,))
            user = c.fetchone()
            if user and check_password_hash(user['password_hash'], p):
                login_user(User(user['id'], user['username']))
                return redirect(url_for('startup'))
            flash('Invalid credentials', 'error')
    return render_template('login.html')

@app.route('/register', methods=['GET', 'POST'])
def register():
    if request.method == 'POST':
        u = request.form.get('username')
        p = request.form.get('password')
        if len(u) < 3 or len(p) < 4:
            flash('Invalid input', 'error')
        else:
            with get_db().cursor() as c:
                try:
                    c.execute('INSERT INTO users (username, password_hash) VALUES (%s, %s)', (u, generate_password_hash(p)))
                    flash('Registration successful! Please log in.', 'success')
                    return redirect(url_for('login'))
                except: flash('User exists', 'error')
    return render_template('register.html')

@app.route('/logout')
@login_required
def logout(): logout_user(); return redirect(url_for('login'))

@app.route('/startup')
@login_required
def startup(): return render_template('startup.html')

@app.route('/api/data')
@login_required
def api_data(): return jsonify(ros2_bridge.get_data())

@app.route('/api/threshold', methods=['POST'])
@login_required
def api_threshold():
    t = int(request.form.get('threshold', 768))
    ros2_bridge.set_threshold(t)
    with get_db().cursor() as c: c.execute("UPDATE settings SET setting_value=%s WHERE setting_key='threshold'", (str(t),))
    return jsonify({'success': True})

@app.route('/api/db/<table>')
@login_required
def api_db(table):
    if table not in ['users', 'settings', 'light_readings']: return jsonify([])
    with get_db().cursor() as c:
        q = "SELECT * FROM light_readings ORDER BY id DESC LIMIT 50" if table == 'light_readings' else f"SELECT * FROM {table}"
        c.execute(q)
        return jsonify(c.fetchall())

if __name__ == '__main__':
    ros2_bridge.start()
    app.run(host='0.0.0.0', port=1337)
