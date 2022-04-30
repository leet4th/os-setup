"""" General
set history=500 " Sets how many lines of history VIM has to remember

" Enable filetype plugins
filetype plugin on
filetype indent on

" Set to auto read when a file is changed from the outside
set autoread
au FocusGained,BufEnter * checktime

""" Mouse
set mousemodel=popup

"""" VIM user interface
set scrolloff=7 " Contexts lines to the cursor when vertically scrolling
set wildmenu    " Command completion help
set cursorline  " Highlight line with cursor
set number      " Display line numbers
set showcmd

" Ignore compiled files
set wildignore=*.o,*~,*.pyc
if has("win16") || has("win32")
    set wildignore+=.git\*,.hg\*,.svn\*
else
    set wildignore+=*/.git/*,*/.hg/*,*/.svn/*,*/.DS_Store
endif

set ruler " Always show current position
set cmdheight=1 " Height of the command bar

" Configure backspace to act properlly
set backspace=indent,eol,start
set whichwrap+=<,>,h,l

set ignorecase " Ignore case when searching
set smartcase  " Will search by case when an upper case is used
set hlsearch   " Highlight search results
set incsearch  " Makes search act like search in modern browsers
set lazyredraw " Don't redraw while executing macros (good performance)
set magic      " For regular expressions turn magic on
set showmatch  " Show matching brackets when text indicator is over them

" No error bells
set noerrorbells
set novisualbell
set visualbell
set t_vb=
set tm=500

"""" Colors and Fonts
syntax enable " Enable syntax highlighting

" Enable 256 colors palette in Gnome Terminal
if $COLORTERM == 'gnome-terminal'
    set t_Co=256
endif

try
    "colorscheme wombat256
    colorscheme xoria256
    "colorscheme feral
    "colorscheme xterm16
    "colorscheme znake
    "colorscheme Atelier_ForestDark
    "colorscheme Dim
    "colorscheme jellybeans
    "colorscheme peaksea
    set background=dark
catch
endtry

"set guifont=Consolas:h9:cANSI:qDRAFT " Set font and size

" Set extra options when running in GUI mode
if has("gui_running")
    set guioptions-=T " Remove toolbar
    set t_Co=256
    set guitablabel=%M\ %t
endif

set encoding=utf8 " Set utf8 as standard encoding and en_US as the standard language

"""" Files and backups cause of windows setup
set nobackup
set nowb
set noswapfile

"""" Text, tabs, and indent
set expandtab " Use spaces instead of tabs
set smarttab
set ai   " Auto indent
set si   " Smart indent
set wrap " Wrap lines

" tab = 4 spaces
set shiftwidth=4
set tabstop=4
set softtabstop=4

" Linebreak on 500 characters
set lbr
set tw=500

" Return to last edit position when opening files (You want this!)
au BufReadPost * if line("'\"") > 1 && line("'\"") <= line("$") | exe "normal! g'\"" | endif

 """ Mapped commands
" With a map leader it's possible to do extra key combinations
" like <leader>w saves the current file
" defulat map leader is "\"
let mapleader = " "

" Fast saving and fast quiting
nmap <leader>w :w!<cr>
nmap <leader>q :q!<cr>
nmap <leader>r :!%:p<cr>

" :W sudo saves the file
" (useful for handling the permission-denied error)
command! W execute 'w !sudo tee % > /dev/null' <bar> edit!

" Useful mappings for managing tabs
map <leader>tn :tabnew<cr>
map <leader>to :tabonly<cr>
map <leader>tc :tabclose<cr>
map <leader>tm :tabmove
map <leader>t  :tabnext<cr>

" Let 'tt' toggle between current and last tab
let g:lasttab = 1
nmap <Leader>tt :exe "tabn ".g:lasttab<CR>
au TabLeave * let g:lasttab = tabpagenr()

" Delete trailing white space on save, useful for some filetypes ;)
fun! CleanExtraSpaces()
    let save_cursor = getpos(".")
    let old_query = getreg('/')
    silent! %s/\s\+$//e
    call setpos('.', save_cursor)
    call setreg('/', old_query)
endfun

map <Leader>c :call CleanExtraSpaces() <cr>

if has("autocmd")
    autocmd BufWritePre *.txt,*.js,*.py,*.wiki,*.sh,*.cc,*.hh,*.c,*.h :call CleanExtraSpaces()
endif

" ss will toggle spell checking
map <leader>ss :setlocal spell!<cr>

" Remove the Windows ^M - when the encodings gets messed up
noremap <Leader>m mmHmt:%s/<C-V><cr>//ge<cr>'tzt'm

" Toggle paste mode on and off
map <leader>pp :setlocal paste!<cr>

"""" Always show the status line
"""set laststatus=2
"""
"""" Format the status line
"""set statusline=\ %{HasPaste()}%F%m%r%h\ %w\ \ CWD:\ %r%{getcwd()}%h\ \ \ Line:\ %l\ \ Column:\ %c
"""
"""" Returns true if paste mode is enabled
"""function! HasPaste()
"""    if &paste
"""        return 'PASTE MODE  '
"""    endif
"""    return ''
"""endfunction



