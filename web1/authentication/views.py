from django.shortcuts import render, redirect
from django.contrib.auth.models import User
from django.contrib import messages
from django.contrib.auth import authenticate, login, logout
from web1 import settings
from django.core.mail import send_mail
from django.contrib.sites.shortcuts import get_current_site
from django.template.loader import render_to_string
from django.utils.encoding import force_bytes, force_str
from django.utils.http import urlsafe_base64_encode, urlsafe_base64_decode
from .tokens import account_activation_token
from django.core.mail import EmailMessage
import logging

# Create your views here.
def home(request):
    return render(request, 'index.html')

def signup(request):
    if request.method == 'POST':
        #username = request.POST.get('username)
        username = request.POST['username']
        fname = request.POST['fname']
        lname = request.POST['lname']
        email = request.POST['email']
        pass1 = request.POST['pass1']
        pass2 = request.POST['pass2']
        
        if User.objects.filter(username=username):
            messages.error(request, "Username already exist! Please try some other username.")
            return redirect('home')
        if User.objects.filter(email=email):
            messages.error(request, "Email already registered! Please use other email.")
            return redirect('home')

        if len(username) > 10:
            messages.error(request, "Username must be less than 10 characters.")
            return redirect('home')

        if pass1!=pass2:
            messages.error(request, "Password didn't match!")

        if not username.isalnum():
            messages.error(request, "Username must be alphanumeric!")
            return redirect('home')
        
        myuser = User.objects.create_user(username, email, pass1)
        myuser.first_name = fname
        myuser.last_name = lname
        myuser.is_active = False

        myuser.save()
        
        messages.success(request, "Your account successfully created. We have sent you a confirmation email! Please confirm your email to finish your registration")
        
        #Welcome email
        subject = "Welcome to GCS"
        message = "We have sent you a confirmation email! Please confirm your email to finish your registration\n"
        from_email = settings.EMAIL_HOST_USER
        to_list = [myuser.email]
        send_mail(subject, message, from_email, to_list, fail_silently = True)

        return redirect("signin")

    return render(request, 'signup.html')
    
def signin(request):

    if request.method =='POST':
        username = request.POST['username']
        pass1 = request.POST['pass1']

        user = authenticate(username=username, password=pass1)
        if user is not None:
            login(request, user)
            return redirect('home')
        else:
            messages.error(request, "Bad credentials!") #signin failed!  
            return redirect('home')

    return render(request, 'signin.html')

def signout(request):
    logout(request)
    messages.success(request, "Logged Out Successfully!")
    return redirect('home')

def activateEmail(request, user, to_email):
    mail_subject = "Activate your user account."
    current_site = get_current_site(request)
    message = render_to_string("activate_email.html",{
        'user': user.username,
        'domain': current_site.domain,
        'uid': urlsafe_base64_encode(force_bytes(user.pk)),
        'token': account_activation_token.make_token(user),
        'protocol': 'https' if request.is_secure() else 'http'
    })

    email = EmailMessage(mail_subject, message, to=[to_email])
    email.fail_silently = False
    #logger = logging.getLogger(__name__)
    try:    
        email.send()
        messages.success(request, f'Dear <b>{user}</b>, please go to you email <b>{to_email}</b> inbox and click on received activation link to confirm and complete the registration. <b>Note:</b> Check your spam folder.')
    except Exception as e:
        #logger.error(f"Failed to send activation email to {to_email}: {e}")
        messages.error(request, f'Problem sending confirmation email to {to_email}, check if you typed it correctly.')

def activate(request, uidb64, token):
    # User = get_user_model()
    try:
        uid = force_str(urlsafe_base64_decode(uidb64))
        myuser = User.objects.get(pk=uid)
    except(TypeError, ValueError, OverflowError, User.DoesNotExist):
        myuser = None

    if myuser is not None and account_activation_token.check_token(myuser, token):
        myuser.is_active = True
        myuser.save()

        messages.success(request, 'Thank you for your email confirmation. Now you can login your account.')
        return redirect('/signin.html')
    else:
        messages.error(request, 'Activation link is invalid!')
    
    return redirect('home')
